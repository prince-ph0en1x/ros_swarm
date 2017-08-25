/**
 * \file animate.cpp
 * \legacy 'turtle_frame.cpp'
 * 
 * \author Aritra Sarkar
 * \date 16-08-2017 (begin)
 */

#include "turtlesim/animate.h"

namespace turtlesim
{

	QtFrame::QtFrame(QWidget* parent, Qt::WindowFlags f)
	: QFrame(parent, f), path_image_(Globals::WORLD_X, Globals::WORLD_Y, QImage::Format_ARGB32), path_painter_(&path_image_), frame_count_(0), id_counter_(0), agt_ctr_(0), obj_ctr_(0), wal_ctr_(0)
	{
		setFixedSize(Globals::WORLD_X, Globals::WORLD_Y);
		setWindowTitle("WanderLaLaLand");
		
		srand(time(NULL));

		update_timer_ = new QTimer(this);
		update_timer_->setInterval(100);
		update_timer_->start();

		connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));
		
		QString images_path = (ros::package::getPath("pas_de_deux") + "/images/").c_str();	// image path
		QVector<QString> icons;
		icons.append("agts.png");
		icons.append("objs.png");
		for (int i = 0; i < icons.size(); ++i) {
			QImage img;
			img.load(images_path + icons[i]);
			icons_images_.append(img);
		}
		
		meter_ = 1;

		szAgt_ = icons_images_[0].height();
		szObj_ = icons_images_[1].height();
		szWal_ = icons_images_[1].height();

		wdAgt_ = Globals::WORLD_X - Globals::GRID_SZ;// - icons_images_[0].width();	// Rendering Limits
		wdObj_ = Globals::WORLD_X - Globals::GRID_SZ;// - icons_images_[1].width();
		wdWal_ = (width() - 1) / szWal_;

		htAgt_ = Globals::WORLD_Y - Globals::GRID_SZ;//icons_images_[0].height();
		htObj_ = Globals::WORLD_Y - Globals::GRID_SZ;//icons_images_[1].height();
		htWal_ = (height() - 1) / szWal_;
		
		rstWrld_srv_	= nh_.advertiseService("resetWorld", &QtFrame::resetWorld, this);
		rstObjs_srv_	= nh_.advertiseService("resetObjects", &QtFrame::resetObjects, this);
		drawWall_srv_	= nh_.advertiseService("drawWall", &QtFrame::drawWall, this);
		drawObjs_srv_	= nh_.advertiseService("drawObjs", &QtFrame::drawObjs, this);
		drawAgts_srv_	= nh_.advertiseService("drawAgts", &QtFrame::drawAgts, this);
		frmUpdt_srv_	= nh_.advertiseService("frmUpdt", &QtFrame::frameUpdate, this);

		kill_srv_ = nh_.advertiseService("kill", &QtFrame::killCallback, this);
	}

	// ############################################## @$ ##############################################
	
	QtFrame::~QtFrame()
	{
		delete update_timer_;
	}

	// ############################################## @$ ##############################################
	
	bool QtFrame::resetWorld(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp)
	{
		turtlesim::QtFrame::resetObjects(req,resp);
		int r = DEFAULT_BG_R;
		int g = DEFAULT_BG_G;
		int b = DEFAULT_BG_B;
		nh_.param("background_r", r, r);
		nh_.param("background_g", g, g);
		nh_.param("background_b", b, b);
		path_image_.fill(qRgb(r, g, b));
		update();
		return true;
	}

	// ############################################## @$ ##############################################
	
	bool QtFrame::resetObjects(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp)
	{
		objs_.clear();
		obj_ctr_ = 0;
		agts_.clear();
		agt_ctr_ = 0;
		return true;
	}

	// ############################################## @$ ##############################################
	
	bool QtFrame::drawWall(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp)
	{
		int i, j;
		for(i = 0; i < Globals::GRID_SZ; i++) {
			for(j = 0; j < Globals::GRID_SZ; j++) {
				if (req.x+i < Globals::WORLD_X && req.y+j < Globals::WORLD_Y) 
					turtlesim::QtFrame::path_image_.setPixel(req.x+i,req.y+j,qRgb(DEFAULT_W_R+(req.x/32+i), DEFAULT_W_G+(req.y/32+j), DEFAULT_W_B));
			}
		}
		update();
		return true;
	}

	// ############################################## @$ ##############################################
	
	bool QtFrame::drawObjs(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp)
	{
		int i, j;
		std::string real_name;
		do {
			std::stringstream ss;
			ss << "obj" << obj_ctr_++;
			real_name = ss.str();
		} while (objs_.find(real_name) != objs_.end());
		TurtlePtr t(new Turtle(ros::NodeHandle(real_name), icons_images_[1], QPointF(req.x, req.y), M_PI/2));
		objs_[real_name] = t;
		resp.name = real_name;
		return true;
	}

	// ############################################## @$ ##############################################
	
	bool QtFrame::drawAgts(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp)
	{
		int i, j;
		std::string real_name;
		do {
			std::stringstream ss;
			ss << "agt" << agt_ctr_++;
			real_name = ss.str();
		} while (agts_.find(real_name) != agts_.end());
		TurtlePtr t(new Turtle(ros::NodeHandle(real_name), icons_images_[0], QPointF(req.x, req.y), M_PI/2));
		agts_[real_name] = t;
		resp.name = real_name;
		return true;
	}

	// ############################################## @$ ##############################################
	
	bool QtFrame::frameUpdate(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp)
	{
		update();
		return true;
	}

	// ############################################## @$ ##############################################

	bool QtFrame::killCallback(turtlesim::Kill::Request& req, turtlesim::Kill::Response&)
	{
		M_Turtle::iterator it = turtles_.find(req.name);
		if (it == turtles_.end()){
			ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
			return false;
		}
		turtles_.erase(it);
		update();
		return true;
	}

	// ############################################## @$ ##############################################

	void QtFrame::onUpdate()
	{
		ros::spinOnce();
		updateTurtles();
		if (!ros::ok())
			close();
	}

	// ############################################## @$ ##############################################

	void QtFrame::paintEvent(QPaintEvent*)
	{
		QPainter painter(this);
		painter.drawImage(QPoint(0, 0), path_image_);
		M_Turtle::iterator it = agts_.begin();
		M_Turtle::iterator end = agts_.end();
		for (; it != end; ++it)
			it->second->paint(painter);
		it = objs_.begin();
		end = objs_.end();
		for (; it != end; ++it)
			it->second->paint(painter);
		it = wals_.begin();
		end = wals_.end();
		for (; it != end; ++it)
			it->second->paint(painter);
	}

	// ############################################## @$ ##############################################

	void QtFrame::updateTurtles()
	{
		if (last_turtle_update_.isZero()) {
			last_turtle_update_ = ros::WallTime::now();
			return;
		}
		bool modified = false;
		M_Turtle::iterator it = agts_.begin();
		M_Turtle::iterator end = agts_.end();  
		for (; it != end; ++it)
			modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, wdAgt_, htAgt_);
		it = objs_.begin();
		end = objs_.end();  
		for (; it != end; ++it)
			modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, wdObj_, htObj_);
		it = wals_.begin();
		end = wals_.end();  
		for (; it != end; ++it)
			modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, wdWal_, htWal_);

		if (modified)
			update();
		++frame_count_;
	}

}

	// ############################################## @$ ##############################################
	
	#include <QApplication>

	class QtAnimate : public QApplication
	{

	public:

		ros::NodeHandlePtr nh_;

		QtAnimate(int& argc, char** argv)
		: QApplication(argc, argv)
		{
			ros::init(argc, argv, "animate", ros::init_options::NoSigintHandler);
			nh_.reset(new ros::NodeHandle);
		}

		int exec()
		{
			turtlesim::QtFrame frame;
			frame.show();
			return QApplication::exec();
		}
	};

	// ############################################## @$ ##############################################

	int main(int argc, char** argv)
	{
		QtAnimate qta(argc, argv);
		return qta.exec();
	}
