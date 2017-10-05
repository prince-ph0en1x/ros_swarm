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
 *     * The name, Aritra Sarkar may not be used to endorse or promote products 
 *       derived from this software without specific prior written permission.
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
 
// Agent Code

#include <stdlib.h>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#endif

#include "turtlesim/globalConfig.h"

enum EnvStates {
	EMPTY = 0, OBSTACLE = 1, AGENT = 2, OBJECT = 3, PATH = 4, EXPLORED = 5
};

enum MoveAgt {
	MOVE_FWD = 1, TURN_LFT = 2, TURN_RHT = 3, TURN_BAK = 4
};

enum MOVE {
	LEFT_MOVE=1, RIGHT_MOVE=2, UP_MOVE=3, DOWN_MOVE=4
};

enum COST {
	LEFT_COST=1, RIGHT_COST=1, UP_COST=1, DOWN_COST=1
};

struct pose {
	int pos[2];	// [0] - X-axis, [1] - Y-axis, [2] - Z-axis (for 3D)
	int ori[1];	// [0] - Azimuth, [1] - Altitude (for 3D)
};

struct node {
	pose p;
	int gCost;	// travelled path cost from source
	int hCost;	// heuristic path cost to sink		// change to tot cost
	std::vector<int> trail;
};

class Agent
{

private:
	int EnvMap[Globals::GRID_Y][Globals::GRID_X];	// 0 - free cell, 1 - wall, 2 - agent, 3xx - object
	pose agt;
	bool pathKnown;
	std::vector<node> nodeQ;
	std::vector<int> setPath;
	int trailCtr;
	int infVal;

public:

	Agent()		// Constructor
	{
		int i,j;
		for(i = 0; i < Globals::GRID_Y; i++) {
			for(j = 0; j < Globals::GRID_X; j++) {
				EnvMap[i][j] = EMPTY;
			}
		}
		infVal = 100000;
	}

	void init(int agtId, int GridWorld[Globals::GRID_Y][Globals::GRID_X])	// seperate function as parameterized constructor is difficult when declaring a looped array of objects.
	{
		int i,j;
		agt.ori[0] = 1;	// 0 - 0/E, 1 - 90/N, 2 - 180/W, 3 - 270/S
		for(i = 0; i < Globals::GRID_Y; i++) {
			for(j = 0; j < Globals::GRID_X; j++) {
				if (GridWorld[i][j] == 200+agtId) {
					EnvMap[i][j] = AGENT;
					agt.pos[0] = j;
					agt.pos[1] = i;
				}
				else if (GridWorld[i][j] == 1 || GridWorld[i][j] >= 300) {	// Walls, Other Agents, Objects
					EnvMap[i][j] = OBSTACLE;
				}

			}
		}
		pathKnown = false;
	}

private:
	
	// ###################################### @$ ######################################

	bool senseF(int GridWorld[Globals::GRID_Y][Globals::GRID_X])	// Front sensor
	{
		switch(agt.ori[0]) {
			case 0:
				if (agt.pos[0]+1 < Globals::GRID_X) {
					if (GridWorld[agt.pos[1]][agt.pos[0]+1] != 1) {
						return true;
					}
				}
				break;
			case 1:
				if (agt.pos[1]-1 > 0) {
					if (GridWorld[agt.pos[1]-1][agt.pos[0]] != 1) {
						return true;
					}
				}
				break;
			case 2:
				if (agt.pos[0]-1 > 0) {
					if (GridWorld[agt.pos[1]][agt.pos[0]-1] != 1) {
						return true;
					}
				}
				break;
			case 3:
				if (agt.pos[1]+1 < Globals::GRID_Y) {
					if (GridWorld[agt.pos[1]+1][agt.pos[0]] != 1) {
						return true;
					}
				}
				break;
		}
		return false;
	}
	
	// ###################################### @$ ######################################

	void actuateF(geometry_msgs::Twist& msg)	// Front actuator
	{
		msg.linear.x = Globals::GRID_SZ;	// move forward
		switch(agt.ori[0]) {
			case 0:
				//msg.linear.x = Globals::GRID_SZ;	// move forward
				agt.pos[0]++;
				break;
			case 1:
				//msg.linear.x = Globals::GRID_SZ;	// move forward
				agt.pos[1]--;
				break;
			case 2:
				//msg.linear.x = Globals::GRID_SZ;	// move forward
				agt.pos[0]--;
				break;
			case 3:
				//msg.linear.x = Globals::GRID_SZ;	// move forward
				agt.pos[1]++;
				break;
		}
	}
	
	// ###################################### @$ ######################################

	bool senseL(int GridWorld[Globals::GRID_Y][Globals::GRID_X])	// Left sensor
	{
		switch(agt.ori[0]) {
			case 0:
				if (agt.pos[1]-1 < Globals::GRID_Y) {
					if (GridWorld[agt.pos[1]-1][agt.pos[0]] != 1) {
						return true;
					}
				}
				break;
			case 1:
				if (agt.pos[0]-1 > 0) {
					if (GridWorld[agt.pos[1]][agt.pos[0]-1] != 1) {
						return true;
					}
				}
				break;
			case 2:
				if (agt.pos[1]+1 > 0) {
					if (GridWorld[agt.pos[1]+1][agt.pos[0]] != 1) {
						return true;
					}
				}
				break;
			case 3:
				if (agt.pos[0]+1 < Globals::GRID_X) {
					if (GridWorld[agt.pos[1]][agt.pos[0]+1] != 1) {
						return true;
					}
				}
				break;
		}
		return false;
	}
	
	// ###################################### @$ ######################################

	void actuateL(geometry_msgs::Twist& msg)	// Left actuator
	{
		msg.angular.z = M_PI / 2.0;	// left rotate
		agt.ori[0] = (agt.ori[0]+1)%4;
	}
	
	// ###################################### @$ ######################################

	bool senseR(int GridWorld[Globals::GRID_Y][Globals::GRID_X])	// Right sensor
	{
		switch(agt.ori[0]) {
			case 0:
				if (agt.pos[1]+1 > 0) {
					if (GridWorld[agt.pos[1]+1][agt.pos[0]] != 1) {
						return true;
					}
				}
				break;
			case 1:
				if (agt.pos[0]+1 < Globals::GRID_X) {
					if (GridWorld[agt.pos[1]][agt.pos[0]+1] != 1) {
						return true;
					}
				}
				break;
			case 2:
				if (agt.pos[1]-1 < Globals::GRID_Y) {
					if (GridWorld[agt.pos[1]-1][agt.pos[0]] != 1) {
						return true;
					}
				}
				break;
			case 3:
				if (agt.pos[0]-1 > 0) {
					if (GridWorld[agt.pos[1]][agt.pos[0]-1] != 1) {
						return true;
					}
				}
				break;
		}
		return false;
	}
	
	// ###################################### @$ ######################################

	void actuateR(geometry_msgs::Twist& msg)	// Right actuator
	{
		msg.angular.z = -M_PI / 2.0;	// right rotate
		agt.ori[0] = (agt.ori[0]-1+4)%4;		// Plus 4 to avoid C++ implementation specific negative number modulus
	}
	
	// ###################################### @$ ######################################

	bool senseB(int GridWorld[Globals::GRID_Y][Globals::GRID_X])	// Back sensor
	{
		switch(agt.ori[0]) {
			case 0:
				if (agt.pos[0]-1 > 0) {
					//if (GridWorld[pos[1]][pos[0]-1] == 0) {		// No back sensing
						return true;
					//}
				}
				break;
			case 1:
				if (agt.pos[1]+1 < Globals::GRID_Y) {
					//if (GridWorld[pos[1]-1][pos[0]] == 0) {		// No back sensing
						return true;
					//}
				}
				break;
			case 2:
				if (agt.pos[0]+1 < Globals::GRID_X) {
					//if (GridWorld[pos[1]][pos[0]-1] == 0) {		// No back sensing
						return true;
					//}
				}
				break;
			case 3:
				if (agt.pos[1]-1 > 0) {
					//if (GridWorld[pos[1]-1][pos[0]] == 0) {		// No back sensing
						return true;
					//}
				}
				break;
		}
		return false;
	}
	
	// ###################################### @$ ######################################

	void actuateB(geometry_msgs::Twist& msg)	// Back actuator
	{
		msg.angular.z = M_PI;	// back rotate
		agt.ori[0] = (agt.ori[0]+2)%4;
	}
	
	// ###################################### @$ ######################################

	bool detectObj(int GridWorld[Globals::GRID_Y][Globals::GRID_X])	// Front sensor for object
	{
		switch(agt.ori[0]) {
			case 0:
				if (agt.pos[0]+1 < Globals::GRID_X) {
					if (GridWorld[agt.pos[1]][agt.pos[0]+1] >= 300) {
						return true;
					}
				}
				break;
			case 1:
				if (agt.pos[1]-1 > 0) {
					if (GridWorld[agt.pos[1]-1][agt.pos[0]] >= 300) {
						return true;
					}
				}
				break;
			case 2:
				if (agt.pos[0]-1 > 0) {
					if (GridWorld[agt.pos[1]][agt.pos[0]-1] >= 300) {
						return true;
					}
				}
				break;
			case 3:
				if (agt.pos[1]+1 < Globals::GRID_Y) {
					if (GridWorld[agt.pos[1]+1][agt.pos[0]] >= 300) {
						return true;
					}
				}
				break;
		}
		return false;
	}
	
	// ###################################### @$ ######################################

	void EnqueNode(int x, int y, int travelled, int heuristic, std::vector<int> trail, int move)	// Insertion Sort
	{
		trail.push_back(move);
		node n1;
		n1.p.pos[0]   = x;
		n1.p.pos[1]   = y;
		n1.gCost = travelled;
		n1.hCost = heuristic;
		n1.trail = trail;
		nodeQ.push_back(n1);	// Add to Open set queue
		
		for(int i = nodeQ.size()-2; i >= 0; i--) {
			if(n1.hCost < nodeQ[i].hCost) {
				nodeQ[i+1].p.pos[0]   = nodeQ[i].p.pos[0];
				nodeQ[i+1].p.pos[1]   = nodeQ[i].p.pos[1];
				nodeQ[i+1].trail = nodeQ[i].trail;
				nodeQ[i+1].gCost = nodeQ[i].gCost;
				nodeQ[i+1].hCost = nodeQ[i].hCost;
				if (i == 0) {
					nodeQ[i].p.pos[0]   = n1.p.pos[0];
					nodeQ[i].p.pos[1]   = n1.p.pos[1];
					nodeQ[i].trail = n1.trail;
					nodeQ[i].gCost = n1.gCost;
					nodeQ[i].hCost = n1.hCost;	
				}
			}
			else {
				nodeQ[i+1].p.pos[0]   = n1.p.pos[0];
				nodeQ[i+1].p.pos[1]   = n1.p.pos[1];
				nodeQ[i+1].trail = n1.trail;
				nodeQ[i+1].gCost = n1.gCost;
				nodeQ[i+1].hCost = n1.hCost;
				break;
			}
		}
	}

	// ###################################### @$ ######################################

	void NextMove(pose p1, pose p2, int travelled, std::vector<int> trail)
	{
		int x = p1.pos[0];
		int y = p1.pos[1];
		int hCost;
		
		if (x > 0 && EnvMap[y][x-1] == EMPTY) {
			//std::cout << nodeQ.size() << "L \n";
			hCost = travelled + LEFT_COST + std::abs(p2.pos[0]-(x-1)) + std::abs(p2.pos[1]-y);
			EnvMap[y][x-1] = EXPLORED;
			if (hCost < infVal)
				EnqueNode(x-1,y,travelled+LEFT_COST,hCost,trail,LEFT_MOVE);
		}

		if(x < Globals::GRID_X-1 && EnvMap[y][x+1] == EMPTY) {
			//std::cout << nodeQ.size() << "R \n";
			hCost = travelled + RIGHT_COST + std::abs(p2.pos[0]-(x+1)) + std::abs(p2.pos[1]-y);
			EnvMap[y][x+1] = EXPLORED;
			if (hCost < infVal)
				EnqueNode(x+1,y,travelled+RIGHT_COST,hCost,trail,RIGHT_MOVE);
		}

		if(y > 0 && EnvMap[y-1][x] == EMPTY) {
			//std::cout << nodeQ.size() << "U \n";
			hCost = travelled + UP_COST + std::abs(p2.pos[0]-x) + std::abs(p2.pos[1]-(y-1));
			EnvMap[y-1][x] = EXPLORED;
			if (hCost < infVal)
				EnqueNode(x,y-1,travelled+UP_COST,hCost,trail,UP_MOVE);
		}
			
		if(y < Globals::GRID_Y-1 && EnvMap[y+1][x] == EMPTY) {
			//std::cout << nodeQ.size() << "D \n";
			hCost = travelled + DOWN_COST + std::abs(p2.pos[0]-x) + std::abs(p2.pos[1]-(y+1));
			EnvMap[y+1][x] = EXPLORED;
			if (hCost < infVal)
				EnqueNode(x,y+1,travelled+DOWN_COST,hCost,trail,DOWN_MOVE);
		}				
	}

	// ###################################### @$ ######################################

	std::vector<int> FindPath(pose p1, pose p2)
	{
		std::vector<int> trail;
		
		EnvMap[p1.pos[1]][p1.pos[0]] = PATH;
		NextMove(p1, p2, 0, trail);
		
		while (nodeQ.size() > 0) {
			//std::cout << nodeQ.size() << "<-- \n";
			node nn = nodeQ.front();
			if(nn.p.pos[0] == p2.pos[0] && nn.p.pos[1] == p2.pos[1] && nn.hCost < infVal) {
				// std::cout << nn.p.x << "," << nn.p.y << " ------>\n";
				trail = nn.trail;
				infVal = nn.hCost;
			}
			nodeQ.erase(nodeQ.begin());
			NextMove(nn.p, p2, nn.gCost, nn.trail);
		}

		for(int i = 0; i < trail.size(); i++) {
			if(trail[i] == LEFT_MOVE) {
				p1.pos[0]--;
			}
			else if(trail[i] == RIGHT_MOVE) {
				p1.pos[0]++;
			}
			else if(trail[i] == UP_MOVE) {
				p1.pos[1]--;
			}
			else if(trail[i] == DOWN_MOVE) {
				p1.pos[1]++;
			}
			else {
				std::cout << "Wrong move\n";
			} 
			EnvMap[p1.pos[1]][p1.pos[0]] = PATH;
		}
		EnvMap[p2.pos[1]][p2.pos[0]] = PATH;

		return trail;
	}
	
public:

	void navigate(int agtId, geometry_msgs::Twist& msg, int GridWorld[Globals::GRID_Y][Globals::GRID_X])
	{
		srand(time(0));
		int i,j,n;
		//std::cout <<"Agent : "<< agtId << " Pose = "<< pos[0]<<","<<pos[1]<<" Dir = " << dir[0] << std::endl;
			
		msg.angular.z = 0;
		msg.linear.x = 0;
		
		/* A* algorithm */
		
		// for each obstacle, calculate path cost by A*
		// allocate the object with least path cost to agent, and actuate
		//printEnvMap();

		if (pathKnown == true) {
			if (trailCtr < setPath.size()) {
				// std::cout << "agt(" << agt.pos[0] <<","<< agt.pos[1] <<"-"<< agt.ori[0] <<" trail:"<< trailCtr <<"="<<setPath[trailCtr] << std::endl;
				if (agt.ori[0] == 0) {
					if (setPath[trailCtr] == RIGHT_MOVE) {
						if (trailCtr != setPath.size()-1)
							actuateF(msg);
						trailCtr++;
					}
					else if (setPath[trailCtr] == UP_MOVE) {
						actuateL(msg);
					}
					else if (setPath[trailCtr] == DOWN_MOVE) {
						actuateR(msg);
					}
					else {
						actuateB(msg);
					}
				}
				else if (agt.ori[0] == 1) {
					if (setPath[trailCtr] == UP_MOVE) {
						if (trailCtr != setPath.size()-1)
							actuateF(msg);
						trailCtr++;
					}
					else if (setPath[trailCtr] == LEFT_MOVE) {
						actuateL(msg);
					}
					else if (setPath[trailCtr] == RIGHT_MOVE) {
						actuateR(msg);
					}
					else {
						actuateB(msg);
					}
				}
				else if (agt.ori[0] == 2) {
					if (setPath[trailCtr] == LEFT_MOVE) {
						if (trailCtr != setPath.size()-1)
							actuateF(msg);
						trailCtr++;
					}
					else if (setPath[trailCtr] == DOWN_MOVE) {
						actuateL(msg);
					}
					else if (setPath[trailCtr] == UP_MOVE) {
						actuateR(msg);
					}
					else {
						actuateB(msg);
					}
				}
				else if (agt.ori[0] == 3) {
					if (setPath[trailCtr] == DOWN_MOVE) {
						if (trailCtr != setPath.size()-1)
							actuateF(msg);
						trailCtr++;
					}
					else if (setPath[trailCtr] == RIGHT_MOVE) {
						actuateL(msg);
					}
					else if (setPath[trailCtr] == LEFT_MOVE) {
						actuateR(msg);
					}
					else {
						actuateB(msg);
					}
				}
				
			}
			return;
		}

		pose obj;
		int cost[Globals::NUM_OBJ];
		int minCost = infVal;
		std::vector<int> trail;
		
		for (n = 0; n < Globals::NUM_OBJ; n++) {
			for(i = 0; i < Globals::GRID_Y; i++) {
				for(j = 0; j < Globals::GRID_X; j++) {
					if (GridWorld[i][j] == 300+n) {
						obj.pos[0] = j;
						obj.pos[1] = i;	// upgrade to multi object
						EnvMap[i][j] = EMPTY;
						nodeQ.clear();
						infVal = 100000;
						trail.clear();
						trail = FindPath(agt,obj);
						cost[n] = trail.size();
						//std::cout << "AGT : (" << agt.pos[0] <<","<< agt.pos[1] << ")"<<" , OBJ : (" << obj.pos[0] <<","<< obj.pos[1] << ") COST : " << cost[n] <<"<"<< minCost << std::endl;
						printEnvMap();
						if (cost[n] < minCost) {
							//std::cout << "-- AGT : " << agtId <<" , OBJ : " << n << " COST : " << cost[n] <<"<"<< minCost << std::endl;
							minCost = cost[n];
							setPath = trail;
							
						}
					}
				}
			}
		}
		trailCtr = 0;
		pathKnown = true;
		
		
		/* Random Walk with Caging
		if (detectObj(GridWorld)) {
			return;	// Agent reached goal
		}
		else {
			bool option[4];
			int options = 0;
			option[0] = senseF(GridWorld);
			option[1] = senseL(GridWorld);
			option[2] = senseR(GridWorld);
			option[3] = senseB(GridWorld);
			for (i = 0; i < 4; i++) {
				if (option[i]) {
					options++;
				}
			}
			if (options == 0)
				return;
			options = floor(options * double(rand())/double(RAND_MAX));
			i = 0;
			while(options != -1) {
				options -= (option[i] ? 1 : 0);
				i++;
			}
			if (i == 1)
				actuateF(msg);
			else if (i == 2)
				actuateL(msg);
			else if (i == 3)
				actuateR(msg);
			else if (i == 4)
				actuateB(msg);	
		}
		*/
		
		/* Direction Oblivious Navigation
		if (senseF(GridWorld))
			actuateF(msg);
		else if (senseL(GridWorld)) {
			if (senseR(GridWorld)) {
				if (double(rand())/double(RAND_MAX) < 0.5)
					actuateR(msg);
				else
					actuateL(msg);
			}
			else {
				actuateL(msg);
			}
		}
		else if (senseR(GridWorld))
			actuateR(msg);
		else if (senseB(GridWorld))
			actuateB(msg);
		*/	
	
		/* Random Nagivation
		if (double(rand())/double(RAND_MAX) < 0.7)	// Turning cost function
			msg.angular.z = (M_PI * floor(3*double(rand())/double(RAND_MAX)) / 2.0) - (M_PI / 2.0); // +180, +90, 0, -90
		else
			msg.angular.z = 0;	
			
		if (msg.angular.z == 0)
			msg.linear.x = Globals::GRID_SZ;
		else
			msg.linear.x = 0;
		*/	
			
		// Pg 111, get parameter
		
	}

	void printEnvMap()
	{
		int i,j;
		for (i = 0; i < Globals::GRID_Y; i++) {
			for (j = 0; j < Globals::GRID_X; j++) {
				if (EnvMap[i][j] == EMPTY) {
					//std::cout << ".";
				}
				else if (EnvMap[i][j] == EXPLORED) {
					//std::cout << ",";
					EnvMap[i][j] = EMPTY;
				}
				else if (EnvMap[i][j] == PATH) {
					//std::cout << "x";
					EnvMap[i][j] = EMPTY;
				}
				else {
					//std::cout << EnvMap[i][j];
				}
			}
			//std::cout << std::endl;
		}
	}
};