// Multi-vehicle multi-object allocation - sequential

//#include <conio>
//using namespace std;

#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define numObj 220
#define numAgt 10

#define xWrld 400
#define yWrld 400
#define zWrld 0

#define maxTime 1000	

uint32_t wrldTime = 0;

/**
 * Define a cartisian coordinate vector for position/velocity 3-tuple 
 */
 
struct vec3
{
	uint16_t x;	// X value 	
	uint16_t y;	// Y value
	uint16_t z;	// Z value
};

/**
 * Define an object's attributes 
 */

struct object
{
	vec3 pos;
	// vec3 vel;
	uint16_t wgt;	// Weight of object
	bool fixd;	// Movable box or obstacle
	bool srvReq;	// Service request/required
};

/**
 * Define an agent's attributes 
 */

struct agent
{
	vec3 pos;
	vec3 vel;
	uint16_t wgt;		// Current carrying weight
	uint16_t wgtMx;	// Maximum weight the rover can carry
	uint16_t load;		// Distance travelled by rover so far
	bool active;		// Agent is active as long as it can be assigned further weights to pick up
};

/**
 * Initialize all objects and agents
 */

void initWorld(object obj[], agent agt[])
{
	int i,j;
	
	// Initialize objects
	for (i=0;i<numObj;i++) {
		obj[i].pos.x = (xWrld * double(rand())) / double(RAND_MAX);
		obj[i].pos.y = (yWrld * double(rand())) / double(RAND_MAX);
		obj[i].pos.z = (zWrld * double(rand())) / double(RAND_MAX);
		obj[i].wgt = (10 * double(rand())) / double(RAND_MAX);
		obj[i].fixd = false;
		obj[i].srvReq = true;
	}
	
	for (i=0;i<numAgt;i++) {
		agt[i].pos.x = (xWrld * double(rand())) / double(RAND_MAX);
		agt[i].pos.y = (yWrld * double(rand())) / double(RAND_MAX);
		agt[i].pos.z = (zWrld * double(rand())) / double(RAND_MAX);
		agt[i].vel.x = 0;
		agt[i].vel.y = 0;
		agt[i].vel.z = 0;
		agt[i].wgt = 0;
		agt[i].wgtMx = 100;
		agt[i].load = 0;
		agt[i].active = true;
	}
}

/**
 * Check if mission is complete
 */

bool checkDone(object obj[])
{
	if (++wrldTime > maxTime) {
		std::cout << "Time Up!" << std::endl;
		return true;
	}
	int i;
	for (i=0;i<numObj;i++) {
		if (obj[i].srvReq == true)
			return false;
	}
	return true;
}

/**
 * Calculate distance between two 3 vec
 */

int calcDist(vec3 p1, vec3 p2)
{
	return sqrt(pow(p2.x - p1.x,2) + pow(p2.y - p1.y,2) + pow(p2.z - p1.z,2));
}

/**
 * Allocation algorithms
 */

void stepWorld(object obj[], agent agt[])
{
	// Find agent which travelled the least distance
	int i, minLoad = 1000000, stepAgt = numAgt+1;
	
	for (i=0;i<numAgt;i++) {
		if (agt[i].active == true && agt[i].load < minLoad) {
			stepAgt = i;
			minLoad = agt[i].load;
			//std::cout << "hi "  << std::endl;
		}	
	}
	
	if (stepAgt == numAgt+1) {
		std::cout << "Time = " << wrldTime << " -- No further assignments possible!" << std::endl;
		wrldTime = maxTime;
		return;
	}	
	
	int minDist = sqrt(pow(xWrld,2) + pow(yWrld,2) + pow(zWrld,2));	// World diagonal
	int stepObj = numObj+1;
	int d;
	
	for (i=0;i<numObj;i++) {
		if (obj[i].srvReq == false || obj[i].fixd == true)
			continue;
		if (obj[i].wgt > (agt[stepAgt].wgtMx - agt[stepAgt].wgt))
			continue;
		d = calcDist(agt[stepAgt].pos,obj[i].pos);
		//std::cout << d  << std::endl;
		if (d < minDist) {
			minDist = d;
			stepObj = i;
		}
	}
	
	if (stepObj == numObj+1) {
		agt[stepAgt].active = false;	// No further assignments possible for agent
		// WATSON : recursive call function?
	}
	else {
		obj[stepObj].srvReq = false;
		agt[stepAgt].wgt += obj[stepObj].wgt;
		agt[stepAgt].load += minDist;
		agt[stepAgt].pos.x = obj[stepObj].pos.x;
		agt[stepAgt].pos.y = obj[stepObj].pos.y;
		agt[stepAgt].pos.z = obj[stepObj].pos.z;
		std::cout << "Time = " << wrldTime << " Agent " << stepAgt << " assigned to Object " << stepObj << " @ (" << obj[stepObj].pos.x << "," << obj[stepObj].pos.y << "," << obj[stepObj].pos.z << ")" << std::endl;
	}
}

/**
 * Display all agents
 */

void logWorld(object obj[], agent agt[])
{
	int i;
		
	for (i=0;i<numAgt;i++) {
		std::cout << "Agent #" << i << "\tPose : (" << agt[i].pos.x << "," << agt[i].pos.y << "," << agt[i].pos.z << ") : (" << agt[i].vel.x <<"," << agt[i].vel.y <<"," << agt[i].vel.z << ")\tWgt : " << agt[i].wgt << "\tLoad : " << agt[i].load << "\tActive : " << agt[i].active << std::endl;
	}

}

int main(int argc, char **argv)
{
	//std::cout << (time(0)) << std::endl;
	srand(time(0));
	
	object obj[numObj];
	agent agt[numAgt];
	
	agt[0].pos.x = 20;
	initWorld(obj,agt);
	
	while (checkDone(obj) == false) {
		if (wrldTime < 2)
			logWorld(obj,agt);
		stepWorld(obj,agt);	
	}
	logWorld(obj,agt);
}


