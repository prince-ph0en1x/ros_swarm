/**
 * \file animate.h
 * \legacy 'turtle_frame.cpp'
 * 
 * \author Aritra Sarkar
 * \date 16-08-2017 (begin)
 */
 
#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/ros.h>

# include <std_srvs/Empty.h>
# include <turtlesim/Spawn.h>
# include <turtlesim/Kill.h>
# include <map>

# include "turtle.h"
#endif

#include <QPointF>
#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#include "turtlesim/globalConfig.h"

// Background Colour
#define DEFAULT_BG_R 0xcf
#define DEFAULT_BG_G 0xcf
#define DEFAULT_BG_B 0xff

// Colour : Wall
#define DEFAULT_W_R 0x32
#define DEFAULT_W_G 0x02
#define DEFAULT_W_B 0x02

namespace turtlesim
{

class QtFrame : public QFrame
{
  Q_OBJECT
public:
  QtFrame(QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~QtFrame();
	
	QImage path_image_;

	ros::ServiceServer rstWrld_srv_;
	ros::ServiceServer rstObjs_srv_;
	ros::ServiceServer drawWall_srv_;
	ros::ServiceServer drawObjs_srv_;
	ros::ServiceServer drawAgts_srv_;
	ros::ServiceServer frmUpdt_srv_;
	bool resetWorld(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);
	bool resetObjects(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);
	bool drawWall(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);
	bool drawObjs(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);
	bool drawAgts(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);

	bool frameUpdate(turtlesim::Spawn::Request&, turtlesim::Spawn::Response&);



protected:
  void paintEvent(QPaintEvent* event);

private slots:
  void onUpdate();

public:
  void updateTurtles();
  bool killCallback(turtlesim::Kill::Request&, turtlesim::Kill::Response&);
  
  
  ros::NodeHandle nh_;
  QTimer* update_timer_;
  QPainter path_painter_;

  uint64_t frame_count_;

  ros::WallTime last_turtle_update_;

  ros::ServiceServer kill_srv_;

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  
  M_Turtle turtles_;
  
  M_Turtle agts_;
  M_Turtle objs_;
  M_Turtle wals_;
  
  uint32_t id_counter_; 
  
  uint32_t agt_ctr_;
  uint32_t obj_ctr_;
  uint32_t wal_ctr_;
  
  QVector<QImage> icons_images_;
  
  QVector<QImage> turtle_images_;
  float meter_;
  float width_in_meters_;
  float height_in_meters_;
  
  // Size, Width and Height of Agent, Object and Wall
  
  float szAgt_;
  float szObj_;
  float szWal_;
  
  float wdAgt_;
  float wdObj_;
  float wdWal_;
  
  float htAgt_;
  float htObj_;
  float htWal_;
  
};

}
