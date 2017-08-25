/**
 * \file pasdedeux.cpp
 * \legacy '9 - PasDeDeuxSense.cpp'
 * 
 * \author Aritra Sarkar
 * \date 16-08-2017 (begin)
 */

#ifndef PAS_DE_DEUX_H
#define PAS_DE_DEUX_H

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <unistd.h>		// for usleep
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>

#include "turtlesim/globalConfig.h"

const int GRID_X = Globals::GRID_X;
const int GRID_Y = Globals::GRID_Y;
const int GRID_A = Globals::GRID_A;

const int GRID_SZ = Globals::GRID_SZ;

const int NUM_OBJ = Globals::NUM_OBJ;
const int NUM_AGT = Globals::NUM_AGT;

const int SIM_DELAY = 10000;

const int INFVAL = 100000;

struct point {
	float x;
	float y;
};

struct pose {
	point pos;
	float ori;	// angle value from 0 rad World East
};

struct object {
	pose p;									// centre of gravity and object facing direction
	pose target;							// target pose
	std::vector<point> pts;
	std::vector<pose> dks;					// docking cells
	bool cSpace[GRID_A][GRID_Y][GRID_X];	// 8 allowed configuration directions
	int wgt;								// weight an object can actuate
	bool pushActive;
	int asgnmnt;							// agtId > objId, objId > tgtId
};

struct node {
	pose p;
	int gCost;	// travelled path cost from source
	int tCost;	// gCost + heuristic path cost to sink
	std::vector<int> trail;		// stores ActionObj sequence
};

enum ActionObj {
	MOVE_E	= 0,
	MOVE_N	= 1,
	MOVE_W	= 2,
	MOVE_S	= 3,
	TURN_C	= 4,
	TURN_AC	= 5,
	SKIP	= 6
};

enum CstActionObj {
	CMOVE_E		= 1,
	CMOVE_N		= 1,
	CMOVE_W		= 1,
	CMOVE_S		= 1,
	CTURN_C		= 2,	// translation preferred over rotation
	CTURN_AC	= 2		// translation preferred over rotation
};

#endif