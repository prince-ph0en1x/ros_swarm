/*
 * Copyright (c) 2017, Aritra Sarkar
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

// roscore
// source ~/Desktop/Aritra_Internship/roscodes/devel/setup.bash
// cd ~/Desktop/Aritra_Internship/roscodes/
// catkin_make
// rosrun turtlebase srmrvr_node
// rosrun turtlebase turtle_autopilot

//namespace turtlesim
//{

turtlesim::PoseConstPtr g_pose;

class TurtleAutopilot
{

private:

	int GridWorld[Globals::GRID_Y][Globals::GRID_X];	// 0 - free cell, 1 - wall, 2xx - agent, 3xx - object 
	geometry_msgs::Twist msg;

public:

	int pilot(int argc, char **argv)
	{
		int i,j,ii,jj,x,y,n;
		bool success;	
		std::ostringstream oss;
		
		srand(time(0));
		
		int numAgt = Globals::NUM_AGT, numObj = Globals::NUM_OBJ;
		if (argc > 2) {
			numAgt = atoi(argv[1]);
			numObj = atoi(argv[2]);
		}
		
		std::cout << "Number of Agents simulated = " << numAgt << std::endl;
		std::cout << "Number of Objects simulated = " << numObj << std::endl;
		
		ros::init(argc, argv, "turtle_autopilot");
		ros::NodeHandle nh;
			
		bool block;
		for(i = 0; i < Globals::GRID_Y; i++) {
			for(j = 0; j < Globals::GRID_X; j++) {
				if (double(rand())/double(RAND_MAX) < 0.1)
					GridWorld[i][j] = 1;
				else
					GridWorld[i][j] = 0;
				if (i == 0 || j == 0 || i == 	Globals::GRID_Y-1 || j == Globals::GRID_X-1)
					GridWorld[i][j] = 1;				
			}
		}
		
		turtlesim::Spawn::Request req;
		turtlesim::Spawn::Response resp;
		
		ros::ServiceClient scW = nh.serviceClient<turtlesim::Spawn>("buildWal");
		for(i = 0; i < Globals::GRID_Y; i++) {
			for(j = 0; j < Globals::GRID_X; j++) {
				if (GridWorld[i][j] == 1) {
					req.x = j * Globals::GRID_SZ;
					req.y = i * Globals::GRID_SZ;
					req.theta = 0;	
					success = scW.call(req, resp);
				}
			}
		}
				
		ros::ServiceClient scA = nh.serviceClient<turtlesim::Spawn>("spawnAgt");
		for(n = 0; n < numAgt; n++) {
			do {
				i = floor(Globals::GRID_Y * double(rand()) / double(RAND_MAX));
				j = floor(Globals::GRID_X * double(rand()) / double(RAND_MAX));
			} while(GridWorld[i][j] != 0);
			GridWorld[i][j] = 200 + n;
			req.x = j * Globals::GRID_SZ;
			req.y = i * Globals::GRID_SZ;//Globals::WORLD_Y - (i+1) * Globals::GRID_SZ;
			req.theta = M_PI / 2;		// Random Angle : 2 * M_PI * double(rand())/double(RAND_MAX);	
			success = scA.call(req, resp);
		}
		
		ros::ServiceClient scO = nh.serviceClient<turtlesim::Spawn>("spawnObj");
		for(n = 0; n < numObj; n++) {
			do {
				i = floor(Globals::GRID_Y * double(rand()) / double(RAND_MAX));
				j = floor(Globals::GRID_X * double(rand()) / double(RAND_MAX));
			} while(GridWorld[i][j] != 0);
			GridWorld[i][j] = 300 + n;
			req.x = j * Globals::GRID_SZ;
			req.y = i * Globals::GRID_SZ;//Globals::WORLD_Y - (i+1) * Globals::GRID_SZ;
			req.theta = 0;	
			success = scO.call(req, resp);
		}
		
		ros::Publisher pb[numAgt];
		ros::Subscriber sb[numAgt];
		Agent agt[numAgt];
		
		for(i = 0; i < numAgt; i++) {
			oss.str("");
			oss << "agt" << i << "/cmd_vel";
			pb[i] = nh.advertise<geometry_msgs::Twist>(oss.str(),1000);
			agt[i].init(i,GridWorld);
			pb[i].publish(msg);
		}
		
		ros::Rate rate(1);	// Important Value - Understand significance on agent angular motion
		rate.sleep();
		while(ros::ok()) {
			for(i = 0; i < numAgt; i++) {
				agt[i].navigate(i,msg,GridWorld);
				pb[i].publish(msg);
				//ROS_INFO_STREAM("Cmd Sent"<<" Linear="<<msg.linear.x<<" Angular="<<msg.angular.z << " Debug" << floor(4*double(rand())/double(RAND_MAX)));
			}
			rate.sleep();
		}
		
		std::cout << "Bye ROSy" << std::endl;
		ros::shutdown();
	}
};

int main(int argc, char** argv)
{
  TurtleAutopilot ap;
  return ap.pilot(argc, argv);
}

//}
