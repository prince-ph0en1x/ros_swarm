/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "turtlesim/globalConfig.h"

#include "turtlesim/turtle_frame.h"

#include <QPointF>

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0xcf	// screen colour
#define DEFAULT_BG_G 0xcf
#define DEFAULT_BG_B 0xff

//bool defEnvMap();

namespace turtlesim
{

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

TurtleFrame::TurtleFrame(QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
, path_image_(Globals::WORLD_X, Globals::WORLD_Y, QImage::Format_ARGB32)
, path_painter_(&path_image_)
, frame_count_(0)
, id_counter_(0)
, agt_ctr_(0)
, obj_ctr_(0)
, wal_ctr_(0)
{
  setFixedSize(Globals::WORLD_X, Globals::WORLD_Y);
  setWindowTitle("WanderLaLaLand");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(100);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_.setParam("background_r", DEFAULT_BG_R);
  nh_.setParam("background_g", DEFAULT_BG_G);
  nh_.setParam("background_b", DEFAULT_BG_B);

  QString images_path = (ros::package::getPath("turtlebase") + "/images/").c_str();	// image path
  
  QVector<QString> icons;
  icons.append("agt.png");	// Agent	: 32x32 px
  icons.append("obj.png");	// Object	: 45x45 px
  icons.append("wal.png");	// Wall	: 128x128 px
  
  for (int i = 0; i < icons.size(); ++i) {
    QImage img;
    img.load(images_path + icons[i]);
    icons_images_.append(img);
  }
  
  meter_ = 1;
  
  szAgt_ = icons_images_[0].height();
  szObj_ = icons_images_[1].height();
  szWal_ = icons_images_[2].height();
  
  wdAgt_ = Globals::WORLD_X - Globals::GRID_SZ;// - icons_images_[0].width();	// Rendering Limits
  wdObj_ = Globals::WORLD_X - Globals::GRID_SZ;// - icons_images_[1].width();
  wdWal_ = (width() - 1) / szWal_;
  
  htAgt_ = Globals::WORLD_Y - Globals::GRID_SZ;//icons_images_[0].height();
  htObj_ = Globals::WORLD_Y - Globals::GRID_SZ;//icons_images_[1].height();
  htWal_ = (height() - 1) / szWal_;

  clear();

  clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);
  
  spawnAgt_srv_ = nh_.advertiseService("spawnAgt", &TurtleFrame::spawnAgtCallback, this);
  spawnObj_srv_ = nh_.advertiseService("spawnObj", &TurtleFrame::spawnObjCallback, this);
  buildWal_srv_ = nh_.advertiseService("buildWal", &TurtleFrame::buildWalCallback, this);
  
  ROS_INFO("Starting turtlesim with node name %s", ros::this_node::getName().c_str()) ;
  
  // spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : Destructor

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : spawnAgtCallback

bool TurtleFrame::spawnAgtCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
	std::string name = spawnAgt(req.name, req.x, req.y, req.theta);
	if (name.empty()) {
		ROS_ERROR("An agent named [%s] already exists", req.name.c_str());
		return false;
	}
	res.name = name;
	return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : spawnAgt

std::string TurtleFrame::spawnAgt(const std::string& name, float x, float y, float angle)
{
	std::string real_name = name;
	if (real_name.empty()) {
		do {
			std::stringstream ss;
			ss << "agt" << agt_ctr_++;
			real_name = ss.str();
		} while (hasAgt(real_name));
	}
	else {
		if (hasAgt(real_name)) {
			return "";
		}
	}

	TurtlePtr t(new Turtle(ros::NodeHandle(real_name), icons_images_[0], QPointF(x, y), angle));
	agts_[real_name] = t;

  int i,j;
  for(i = - Globals::GRID_SZ / 2; i < Globals::GRID_SZ / 2; i++) {
    for(j = - Globals::GRID_SZ / 2; j < Globals::GRID_SZ / 2; j++) {
      if (x+i > 0 && y+j > 0 && x+i < Globals::WORLD_X && y+j < Globals::WORLD_Y)
        turtlesim::TurtleFrame::path_image_.setPixel(x+i,y+j,qRgb(0x7f, 0xff, 0x7f));
    }
  }

	update();
	ROS_INFO("Spawning agent [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);
	return real_name;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : hasAgt

bool TurtleFrame::hasAgt(const std::string& name)
{
	return agts_.find(name) != agts_.end();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : spawnObjCallback

bool TurtleFrame::spawnObjCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
	std::string name = spawnObj(req.name, req.x, req.y, req.theta);
	if (name.empty()) {
		ROS_ERROR("An object named [%s] already exists", req.name.c_str());
		return false;
	}
	res.name = name;
	return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : spawnObj

std::string TurtleFrame::spawnObj(const std::string& name, float x, float y, float angle)
{
	std::string real_name = name;
	if (real_name.empty()) {
		do {
			std::stringstream ss;
			ss << "obj" << obj_ctr_++;
			real_name = ss.str();
		} while (hasObj(real_name));
	}
	else {
		if (hasObj(real_name)) {
			return "";
		}
	}

	//TurtlePtr t(new Turtle(ros::NodeHandle(real_name), icons_images_[1], QPointF(x, y), angle));
	//objs_[real_name] = t;
	
  int i,j;
  for(i = - Globals::GRID_SZ / 2; i < Globals::GRID_SZ / 2; i++) {
    for(j = - Globals::GRID_SZ / 2; j < Globals::GRID_SZ / 2; j++) {
      if (x+i > 0 && y+j > 0 && x+i < Globals::WORLD_X && y+j < Globals::WORLD_Y)
        turtlesim::TurtleFrame::path_image_.setPixel(x+i,y+j,qRgb(0xff, 0x7f, 0x7f));
    }
  }

  //update();
	ROS_INFO("Spawning object [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);
	return real_name;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : hasObj

bool TurtleFrame::hasObj(const std::string& name)
{
	return objs_.find(name) != objs_.end();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : buildWalCallback

bool TurtleFrame::buildWalCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
	int i,j;
	for(i = - Globals::GRID_SZ / 2; i < Globals::GRID_SZ / 2; i++) {
		for(j = - Globals::GRID_SZ / 2; j < Globals::GRID_SZ / 2; j++) {
			if (req.x+i > 0 && req.y+j > 0 && req.x+i < Globals::WORLD_X && req.y+j < Globals::WORLD_Y)
				turtlesim::TurtleFrame::path_image_.setPixel(req.x+i,req.y+j,qRgb(0x8f, 0x8f, 0xff));
		}
	}
	
	/*std::string name = buildWal(req.name, req.x, req.y, req.theta);
	if (name.empty()) {
		ROS_ERROR("A wall segment named [%s] already exists", req.name.c_str());
		return false;
	}*/
	res.name = req.name;
	return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : buildWal

std::string TurtleFrame::buildWal(const std::string& name, float x, float y, float angle)
{
	std::string real_name = name;
	if (real_name.empty()) {
		do {
			std::stringstream ss;
			ss << "wal" << wal_ctr_++;
			real_name = ss.str();
		} while (hasWal(real_name));
	}
	else {
		if (hasWal(real_name)) {
			return "";
		}
	}
	TurtlePtr t(new Turtle(ros::NodeHandle(real_name), icons_images_[2], QPointF(x, htWal_ - y), angle)); // Conversion from BottomLeft Origin to TopLeft Origin
	wals_[real_name] = t;
	update();
	ROS_INFO("Building wall [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);
	return real_name;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : hasWal

bool TurtleFrame::hasWal(const std::string& name)
{
	return wals_.find(name) != wals_.end();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool TurtleFrame::spawnCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool TurtleFrame::killCallback(turtlesim::Kill::Request& req, turtlesim::Kill::Response&)
{
  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end())
  {
    ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

  TurtlePtr t(new Turtle(ros::NodeHandle(real_name), turtle_images_[index], QPointF(x, height_in_meters_ - y), angle));
  turtles_[real_name] = t;
  update();

  ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void TurtleFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

  nh_.param("background_r", r, r);
  nh_.param("background_g", g, g);
  nh_.param("background_b", b, b);

  path_image_.fill(qRgb(r, g, b));
  
  update();
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void TurtleFrame::onUpdate()
{
  ros::spinOnce();

  updateTurtles();

  if (!ros::ok())
  {
    close();
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : paintEvent

void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
  
	it = agts_.begin();
	end = agts_.end();
	for (; it != end; ++it) {
		it->second->paint(painter);
	}
	  
	it = objs_.begin();
	end = objs_.end();
	for (; it != end; ++it) {
		it->second->paint(painter);
	}
	  
	it = wals_.begin();
	end = wals_.end();
	for (; it != end; ++it) {
		it->second->paint(painter);
	}
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ : updateTurtles

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  bool modified = false;
  
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  
	it = agts_.begin();
	end = agts_.end();  
	for (; it != end; ++it) {
		modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, wdAgt_, htAgt_);
	}
	
	it = objs_.begin();
	end = objs_.end();  
	for (; it != end; ++it) {
		modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, wdObj_, htObj_);
	}
	
	it = wals_.begin();
	end = wals_.end();  
	for (; it != end; ++it) {
		modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, wdWal_, htWal_);
	}
  
  if (modified)
  {
    update();
  }

  ++frame_count_;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing turtlesim.");
  clear();
  return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ @$ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}
