
// ROS version of Hello World

//#include <QFrame>

#include <stdlib.h>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
//#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>

#include "turtlesim/globalConfig.h"
#include "turtlesim/turtle_frame.h"
# include <boost/shared_ptr.hpp>

//#include "turtlesim/agent.h"
#include "agent.cpp"

velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Turtle::velocityCallback, this);
  
  
  
