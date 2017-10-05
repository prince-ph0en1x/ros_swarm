/**
 * \file cagingMove.cpp
 * 
 * Internship Project at TCS Innovation Labs Kolkata
 * 
 * \author Aritra Sarkar
 */

/* TBD
	-	Rotation selectDocks
	-	Agent Sequential updateDocks
	-	Agent target to Object C.G.
	-	Precalculate object trajectory taking 1 rover docking trajectory into account
*/

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <unistd.h>		// for usleep
 
const int GRID_X = 40;
const int GRID_Y = 40;
const int GRID_A = 8;
const int NUM_OBJ = 3;
const int NUM_AGT = 2;
const int SIM_TIME = 1;

enum EnvStates {
	EMPTY		= 0, 
	OBSTACLE	= 1, 
	AGENT		= 2, 
	OBJECT		= 3, 
	PATH		= 4, 
	EXPLORED	= 5, 
	TARGET		= 6, 
	DOCK		= 7, 
	OBJCG		= 8, 
	POC			= 9,
	AGENTCG		= 10,	
};

enum ActionObj {
	MOVE_E	= 0,
	MOVE_N	= 1,
	MOVE_W	= 2,
	MOVE_S	= 3,
	TURN_C	= 4,
	TURN_AC	= 5
};

enum CstActionObj {
	CMOVE_E		= 1,		// CHANGE IF DIRN CHANGE
	CMOVE_N		= 1,
	CMOVE_W		= 1,
	CMOVE_S		= 1,
	CTURN_C		= 2,
	CTURN_AC	= 2
};

struct point {
	float x;
	float y;
};

struct pose {
	point pos;
	float ori;	// angle value from 0 rad World East
};

struct object {
	pose p;			// centre of gravity and object facing direction
	pose target;	// target pose
	std::vector<point> pts;
	std::vector<pose> dks;	// docking cells
	bool cSpace[GRID_A][GRID_Y][GRID_X];		// 8 allowed configuration directions
	int wgt;		// weight of an object / weight an agent can manipulate
	bool pushActive;
};

struct node {
	pose p;
	int gCost;	// travelled path cost from source
	int tCost;	// gCost + heuristic path cost to sink
	std::vector<int> trail;		// stores ActionObj sequence
};


class CagingTestbed
{

public:

	int EnvMap[GRID_Y][GRID_X];	// 0 - free cell, 1 - wall, 2 - agent, 3xx - object
	int TmpMap[GRID_Y][GRID_X];	// 0 - free cell, 1 - wall, 2 - agent, 3xx - object
	object obj[NUM_OBJ];
	object agt[NUM_AGT];
	std::vector<node> nodeQ;
	int infVal;

	// ############################################## @$ ##############################################
	
	CagingTestbed()
	{
		std::cout << "\033[2J\033[1;1H";
		int i,j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				EnvMap[i][j] = EMPTY;
			}
		}
		infVal = 100000;
	}
	
	// ############################################## @$ ##############################################

	void cage()
	{
		int t,i,j,n;
		std::vector<int> agttrail[NUM_AGT];
		std::vector<int> objtrail[NUM_OBJ];
		pose tmp;
		
		initWall();	
		initObjects();
		initAgents();
		
		// Build World Environment Map for Display (upgrade to pixel based image)
		
		for (n = 0; n < NUM_OBJ; n++) {
			rotateObject(&obj[n],n);
		}
		for (n = 0; n < NUM_AGT; n++) {
			rotateObject(&agt[n],0);
		}
		//printEnvMap();

		for (t = 0; t < SIM_TIME; t++) {	// For Each Time Step

			// Construct Configuration Space
			//std::cout << "Building Configuration Space" << std::endl;
			for (n = 0; n < NUM_OBJ; n++) {
				for (i = 0; i < GRID_Y; i++) {
					for (j = 0; j < GRID_X; j++) {
						TmpMap[i][j] = EnvMap[i][j];
					}
				}
				for (i = 0; i < GRID_A; i++) {
					buildCSpace(&obj[n],obj[n].p.ori);
					rotateObject(&obj[n],1);
				}
				for (i = 0; i < GRID_Y; i++) {
					for (j = 0; j < GRID_X; j++) {
						EnvMap[i][j] = TmpMap[i][j];
					}
				}
				updateDocks(&obj[n]);
				selectDocks(&obj[n]);
			}
			
			for (n = 0; n < NUM_AGT; n++) {
				for (i = 0; i < GRID_A; i++) {
					buildCSpace(&agt[n],agt[n].p.ori);
					rotateObject(&agt[n],1);
				}
				updateDocks(&agt[n]);
			}
			//printCSpace(&agt[0],2);
			
			// 3D A* Search for path
			//std::cout << "Calculating Object Trails" << std::endl;
			for (n = 0; n < NUM_OBJ; n++) {
				objtrail[n].clear();
				objtrail[n] = pathObject(n);
			}
			
			for (n = 0; n < 1; n++) {
				if (objtrail[n].size() == 0)
					continue;
				tmp.pos.x = obj[n].target.pos.x;
				tmp.pos.y = obj[n].target.pos.y;
				tmp.ori = obj[n].target.ori;
				obj[n].target.pos.x = obj[n].p.pos.x;
				obj[n].target.pos.y = obj[n].p.pos.y;
				obj[n].target.ori = obj[n].p.ori;
				switch(objtrail[n].front()) {
					case MOVE_E		: 
						obj[n].target.pos.x++;
						break;
					case MOVE_N		: 
						obj[n].target.pos.y--;
						break;
					case MOVE_W		: 
						obj[n].target.pos.x--;
						break;
					case MOVE_S		:
						obj[n].target.pos.y++;
						break;
					case TURN_C		: 
						obj[n].target.ori--;
						break;
					case TURN_AC	: 
						obj[n].target.ori++;
						break;
				}
				selectDocks(&obj[n]);
				obj[n].target.pos.x = tmp.pos.x;
				obj[n].target.pos.y = tmp.pos.y;
				obj[n].target.ori = tmp.ori;
			}
				
			// pair agents with objects
			
			//std::cout << "Calculating Agent Trails" << std::endl;
			for (n = 0; n < NUM_AGT; n++) {
				agttrail[n].clear();
				agttrail[n] = pathAgent(n,0);	// agtId, objId
			}
			//printTrail(agttrail[0]);
			
			usleep(100000);
			std::cout << "\033[2J\033[1;1H";
			std::cout << "Simulation Time " << t << std::endl;
			printEnvMap();
			
			//agttrail[0].clear();
			//agttrail[1].clear();

			if (agttrail[0].size() > 1) {
				switch(agttrail[0].front()) {
					case MOVE_E		: translateObject(&agt[0],1,0); break;
					case MOVE_N		: translateObject(&agt[0],0,-1); break;
					case MOVE_W		: translateObject(&agt[0],-1,0); break;
					case MOVE_S		: translateObject(&agt[0],0,1); break;
					case TURN_C		: rotateObject(&agt[0],-1); break;
					case TURN_AC	: rotateObject(&agt[0],1); break;
				}
				continue;
			}
			
			if (agttrail[1].size() > 1) {
				switch(agttrail[1].front()) {
					case MOVE_E		: translateObject(&agt[1],1,0); break;
					case MOVE_N		: translateObject(&agt[1],0,-1); break;
					case MOVE_W		: translateObject(&agt[1],-1,0); break;
					case MOVE_S		: translateObject(&agt[1],0,1); break;
					case TURN_C		: rotateObject(&agt[1],-1); break;
					case TURN_AC	: rotateObject(&agt[1],1); break;
				}
				continue;
			}

			// move object based on docking point
			if (objtrail[0].size() > 1) {
				switch(objtrail[0].front()) {
					case MOVE_E		: 
						translateObject(&obj[0],1,0); 
						//translateObject(&agt[0],1,0); 
						//translateObject(&agt[1],1,0); 
						break;
					case MOVE_N		: 
						translateObject(&obj[0],0,-1); 
						//translateObject(&agt[0],0,-1); 
						//translateObject(&agt[1],0,-1); 
						break;
					case MOVE_W		: 
						translateObject(&obj[0],-1,0); 
						//translateObject(&agt[0],-1,0); 
						//translateObject(&agt[1],-1,0); 
						break;
					case MOVE_S		: 
						translateObject(&obj[0],0,1); 
						//translateObject(&agt[0],0,1); 
						//translateObject(&agt[1],0,1); 
						break;
					case TURN_C		: rotateObject(&obj[0],-1); break;
					case TURN_AC	: rotateObject(&obj[0],1); break;
				}
				continue;
			}
			break;
							
		}	
		//usleep(100000);
		//std::cout << "\033[2J\033[1;1H";
		//std::cout << "Simulation End Time " << t << std::endl;
		printEnvMap();
				
	}

	// ############################################## @$ ##############################################
	
	std::vector<int> pathAgent(int agtId, int objId)
	{
		int i,j;
		std::vector<int> trail;
		nodeQ.clear();
		infVal = 100000;
		// temporary store cSpace and restore

		pose p1,p2;	// agt_cg, obj_cg
		p1.pos.x = (int)round(agt[agtId].p.pos.x);
		p1.pos.y = (int)round(agt[agtId].p.pos.y);
		p1.ori = (agt[agtId].p.ori);
		p2.pos.x = (int)round(obj[objId].p.pos.x);
		p2.pos.y = (int)round(obj[objId].p.pos.x);
		p2.ori = (obj[objId].p.ori);

		NextMove(&agt[agtId], p1, p2, 0, trail);
			
		while (nodeQ.size() > 0) {
			node nn = nodeQ.front();
			if(nn.tCost < infVal) {
				for (j = 0; j < agt[agtId].dks.size(); j++) {
					for (i = 0; i < obj[objId].dks.size(); i++) {
						if (nn.p.pos.x-p1.pos.x+agt[agtId].dks[j].pos.x == obj[objId].dks[i].pos.x && nn.p.pos.y-p1.pos.y+agt[agtId].dks[j].pos.y == obj[objId].dks[i].pos.y){// && round(std::abs(agt[agtId].dks[j].ori-obj[objId].dks[i].ori)) == 4 ) {
							trail = nn.trail;
							infVal = nn.tCost;
							j = agt[agtId].dks.size();
							break;
						}
					}
				}
			}
			nodeQ.erase(nodeQ.begin());
			NextMove(&agt[agtId], nn.p, p2, nn.gCost, nn.trail);
		}
		return trail;
	}

	// ############################################## @$ ##############################################
	
	void initWall()
	{
		int k,ii,jj;
		
		for (k = 0; k < GRID_X; k++) {
			EnvMap[0][k] = OBSTACLE;
			EnvMap[1][k] = OBSTACLE;
			EnvMap[GRID_Y-1][k] = OBSTACLE;
			EnvMap[GRID_Y-2][k] = OBSTACLE;
		}
		for (k = 0; k < GRID_Y; k++) {
			EnvMap[k][0] = OBSTACLE;
			EnvMap[k][1] = OBSTACLE;
			EnvMap[k][GRID_X-1] = OBSTACLE;
			EnvMap[k][GRID_X-2] = OBSTACLE;
		}
		for (k = GRID_X/2; k < GRID_X; k++) {
			EnvMap[GRID_Y/2+8][k] = OBSTACLE;
			EnvMap[GRID_Y/2+9][k] = OBSTACLE;
		}
		/*for (k = GRID_Y/2; k < GRID_Y; k++) {
			EnvMap[k][GRID_X/2+3] = OBSTACLE;
			EnvMap[k][GRID_X/2+4] = OBSTACLE;
		}*/		
	}
	
	// ############################################## @$ ##############################################
	
	void initAgents()
	{
		int i,j;
		point z;

		// Agent 1
		for (i = 30; i < 30+3; i++) {
			for (j = 4; j < 4+3; j++) {
				z.x = j;
				z.y = i;
				agt[1].pts.push_back(z);
			}
		}
		agt[1].p.pos.x = 5;
		agt[1].p.pos.y = 31;
		agt[1].p.ori = 2;
		agt[1].wgt = 1;
		agt[1].pushActive = false;

		// Agent 0	
		for (i = 30; i < 30+3; i++) {
			for (j = 15; j < 15+3; j++) {
				z.x = j;
				z.y = i;
				agt[0].pts.push_back(z);
			}
		}
		agt[0].p.pos.x = 16;
		agt[0].p.pos.y = 31;
		agt[0].p.ori = 2;
		agt[0].wgt = 1;
		agt[0].pushActive = false;
	}

	// ############################################## @$ ##############################################
	
	void printTrail(std::vector<int> trail)
	{
		int n;
		for (n = 0; n < trail.size(); n++) {
			if (trail[n] == MOVE_W)
				std::cout << "Step " << (n+1) << " --> MOVE_W" << std::endl;
			if (trail[n] == MOVE_E)
				std::cout << "Step " << (n+1) << " --> MOVE_E" << std::endl;
			if (trail[n] == MOVE_N)
				std::cout << "Step " << (n+1) << " --> MOVE_N" << std::endl;
			if (trail[n] == MOVE_S)
				std::cout << "Step " << (n+1) << " --> MOVE_S" << std::endl;
			if (trail[n] == TURN_C)
				std::cout << "Step " << (n+1) << " --> TURN_C" << std::endl;
			if (trail[n] == TURN_AC)
				std::cout << "Step " << (n+1) << " --> TURN_AC" << std::endl;
		}
	}

	// ############################################## @$ ##############################################
	
	void initObjects()
	{
		int i,j;
		point z;

		// Object 0
		for (i = 15; i < 15+7; i++) {
			for (j = 8; j < 8+13; j++) {
				z.x = j;
				z.y = i;
				obj[0].pts.push_back(z);
			}
		}
		obj[0].p.pos.x = 14;
		obj[0].p.pos.y = 18;
		obj[0].p.ori = 0;
		obj[0].target.pos.x = 28;
		obj[0].target.pos.y = 10;
		obj[0].target.ori = 0;
		obj[0].wgt = 2;
		obj[0].pushActive = false;

		/*
		// Object 1
		for (i = 8; i < 8+7; i++) {
			z.y = i;
			z.x = 24;
			obj[1].pts.push_back(z);
			z.x = 34;
			obj[1].pts.push_back(z);
		}
		for (j = 24; j < 24+10; j++) {
			z.x = j;
			z.y = 14;
			obj[1].pts.push_back(z);
		}
		obj[1].p.pos.x = 29;
		obj[1].p.pos.y = 11;
		obj[1].p.ori = 0;
		obj[1].target.pos.x = 7;
		obj[1].target.pos.y = 7;
		obj[1].target.ori = 0;
		obj[1].wgt = 1;

		// Object 2
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (distC(i,j,30,30) < 5) {
					z.x = j;
					z.y = i;
					obj[2].pts.push_back(z);
				}
			}
		}
		obj[2].p.pos.x = 30;
		obj[2].p.pos.y = 30;
		obj[2].p.ori = 0;
		obj[2].target.pos.x = 7;
		obj[2].target.pos.y = 7;
		obj[2].target.ori = 0;
		obj[2].wgt = 3;
		*/
	}
	
	// ############################################## @$ ##############################################
	
	void EnqueNode(int x, int y, int th, int travelled, int total, std::vector<int> trail, int move)	// Insertion Sort
	{
		trail.push_back(move);
		node n1;
		n1.p.pos.x   = x;
		n1.p.pos.y   = y;
		n1.p.ori = th;
		n1.gCost = travelled;
		n1.tCost = total;
		n1.trail = trail;
		nodeQ.push_back(n1);	// Add to Open set queue
		
		for(int i = nodeQ.size()-2; i >= 0; i--) {
			if(n1.tCost < nodeQ[i].tCost) {
				nodeQ[i+1].p.pos.x   = nodeQ[i].p.pos.x;
				nodeQ[i+1].p.pos.y   = nodeQ[i].p.pos.y;
				nodeQ[i+1].p.ori   = nodeQ[i].p.ori;
				nodeQ[i+1].trail = nodeQ[i].trail;
				nodeQ[i+1].gCost = nodeQ[i].gCost;
				nodeQ[i+1].tCost = nodeQ[i].tCost;
				if (i == 0) {
					nodeQ[i].p.pos.x   = n1.p.pos.x;
					nodeQ[i].p.pos.y   = n1.p.pos.y;
					nodeQ[i].p.ori   = n1.p.ori;
					nodeQ[i].trail = n1.trail;
					nodeQ[i].gCost = n1.gCost;
					nodeQ[i].tCost = n1.tCost;	
				}
			}
			else {
				nodeQ[i+1].p.pos.x   = n1.p.pos.x;
				nodeQ[i+1].p.pos.y   = n1.p.pos.y;
				nodeQ[i+1].p.ori   = n1.p.ori;
				nodeQ[i+1].trail = n1.trail;
				nodeQ[i+1].gCost = n1.gCost;
				nodeQ[i+1].tCost = n1.tCost;
				break;
			}
		}
	}

	// ############################################## @$ ##############################################
	
	void NextMove(object* o, pose p1, pose p2, int travelled, std::vector<int> trail)
	{
		int x = p1.pos.x;
		int y = p1.pos.y;
		int th = p1.ori;
		int tCost;
		
		if (x > 0 && o->cSpace[th][y][x-1]) {
			tCost = travelled + CMOVE_W + std::abs(p2.pos.x-(x-1)) + std::abs(p2.pos.y-y) + std::abs(p2.ori-th);
			o->cSpace[th][y][x-1] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x-1,y,th,travelled+CMOVE_W,tCost,trail,MOVE_W);
		}

		if (x < GRID_X-1 && o->cSpace[th][y][x+1]) {
			tCost = travelled + CMOVE_E + std::abs(p2.pos.x-(x+1)) + std::abs(p2.pos.y-y) + std::abs(p2.ori-th);
			o->cSpace[th][y][x+1] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x+1,y,th,travelled+CMOVE_E,tCost,trail,MOVE_E);
		}

		if (y > 0 && o->cSpace[th][y-1][x]) {
			tCost = travelled + CMOVE_N + std::abs(p2.pos.x-x) + std::abs(p2.pos.y-(y-1)) + std::abs(p2.ori-th);
			o->cSpace[th][y-1][x] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x,y-1,th,travelled+CMOVE_N,tCost,trail,MOVE_N);
		}

		if (y < GRID_Y-1 && o->cSpace[th][y+1][x]) {
			tCost = travelled + CMOVE_S + std::abs(p2.pos.x-x) + std::abs(p2.pos.y-(y+1)) + std::abs(p2.ori-th);
			o->cSpace[th][y+1][x] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x,y+1,th,travelled+CMOVE_S,tCost,trail,MOVE_S);
		}

		if (th > 0 && o->cSpace[th-1][y][x]) {
			tCost = travelled + CTURN_C + std::abs(p2.pos.x-x) + std::abs(p2.pos.y-y) + std::abs(p2.ori-(th-1));
			o->cSpace[th-1][y][x] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x,y,th-1,travelled+CTURN_C,tCost,trail,TURN_C);
		}

		if (th < GRID_A-1 && o->cSpace[th+1][y][x]) {
			tCost = travelled + CTURN_AC + std::abs(p2.pos.x-x) + std::abs(p2.pos.y-y) + std::abs(p2.ori-(th+1));
			o->cSpace[th+1][y][x] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x,y,th+1,travelled+CTURN_AC,tCost,trail,TURN_AC);
		}

	}

	// ############################################## @$ ##############################################

	std::vector<int> pathObject(int objId)
	{
		std::vector<int> trail;
		nodeQ.clear();
		infVal = 100000;

		pose p1,p2;	// cg, target
		p1.pos.x = (int)round(obj[objId].p.pos.x);
		p1.pos.y = (int)round(obj[objId].p.pos.y);
		p1.ori = (obj[objId].p.ori);//(int)round
		p2.pos.x = (int)round(obj[objId].target.pos.x);
		p2.pos.y = (int)round(obj[objId].target.pos.y);
		p2.ori = (int)round(obj[objId].target.ori);

		//std::cout << "CG (" <<p1.pos.x<<","<<p1.pos.y<<","<<p1.ori<<") XX ("<<p2.pos.x<<","<<p2.pos.y<<","<<p2.ori<<")" << std::endl;
		
		NextMove(&obj[objId], p1, p2, 0, trail);
		
		while (nodeQ.size() > 0) {
			//std::cout << nodeQ.size() << "<-- \n";
			node nn = nodeQ.front();
			if(nn.p.pos.x == p2.pos.x && nn.p.pos.y == p2.pos.y && nn.p.ori == p2.ori  && nn.tCost < infVal) {
				//std::cout << nn.p.pos.x << "," << nn.p.pos.y << " ------>\n";
				trail = nn.trail;
				infVal = nn.tCost;
			}
			nodeQ.erase(nodeQ.begin());
			NextMove(&obj[objId], nn.p, p2, nn.gCost, nn.trail);
		}
		return trail;
	}

	// ############################################## @$ ##############################################

	void updateDocks(object* o)
	{
		int i,j,k;
		pose d;

		for (k = 0; k < o->dks.size();) {			
			i = o->dks[k].pos.y;
			j = o->dks[k].pos.x;
			o->dks.erase(o->dks.begin()+k);
			if (EnvMap[i][j] == DOCK)
				EnvMap[i][j] = OBJECT;
		}

		for (k = 0; k < o->pts.size(); k++) {
			i = round(o->pts[k].y);
			j = round(o->pts[k].x);
			d.pos.y = i;
			d.pos.x = j;
			if (i-1 > 0 && EnvMap[i-1][j] == EMPTY) {
				d.ori = 3*GRID_A/4;						// rover should face down to dock/push
				o->dks.push_back(d);
				EnvMap[i][j] = DOCK;
			}
			if (i+1 < GRID_Y && EnvMap[i+1][j] == EMPTY) {
				d.ori = GRID_A/4;
				o->dks.push_back(d);
				EnvMap[i][j] = DOCK;
			}
			if (j-1 > 0 && EnvMap[i][j-1] == EMPTY) {
				d.ori = 0;
				o->dks.push_back(d);
				EnvMap[i][j] = DOCK;
			}
			if (j+1 < GRID_X && EnvMap[i][j+1] == EMPTY) {
				d.ori = GRID_A/2;
				o->dks.push_back(d);
				EnvMap[i][j] = DOCK;
			}
		}
	}

	// ############################################## @$ ##############################################

	void selectDocks(object* o)
	{
		int i,j,k;
		for (k = 0; k < o->dks.size(); k++) {			
			i = round(o->dks[k].pos.y);
			j = round(o->dks[k].pos.x);
			if (o->dks[k].ori == 0) {
				if (distM(o->p.pos.x+1,o->p.pos.y,o->target.pos.x,o->target.pos.y) >= distM(o->p.pos.x,o->p.pos.y,o->target.pos.x,o->target.pos.y)) {
					o->dks.erase(o->dks.begin()+k);
					k--;
					EnvMap[i][j] = OBJECT;
				}
			}
			else if (o->dks[k].ori == GRID_A/4) {
				if (distM(o->p.pos.x,o->p.pos.y-1,o->target.pos.x,o->target.pos.y) >= distM(o->p.pos.x,o->p.pos.y,o->target.pos.x,o->target.pos.y)) {
					o->dks.erase(o->dks.begin()+k);
					k--;
					EnvMap[i][j] = OBJECT;
				}
			}
			else if (o->dks[k].ori == GRID_A/2) {
				if (distM(o->p.pos.x-1,o->p.pos.y,o->target.pos.x,o->target.pos.y) >= distM(o->p.pos.x,o->p.pos.y,o->target.pos.x,o->target.pos.y)) {
					o->dks.erase(o->dks.begin()+k);
					k--;
					EnvMap[i][j] = OBJECT;
				}
			}
			else if (o->dks[k].ori == 3*GRID_A/4) {
				if (distM(o->p.pos.x,o->p.pos.y+1,o->target.pos.x,o->target.pos.y) >= distM(o->p.pos.x,o->p.pos.y,o->target.pos.x,o->target.pos.y)) {
					o->dks.erase(o->dks.begin()+k);
					k--;
					EnvMap[i][j] = OBJECT;
				}
			}
		}
	}

	// ############################################## @$ ##############################################

	void rotateObject(object* o, int rot)
	{
		float k,ii,jj;
		o->p.ori = std::fmod(o->p.ori+rot,GRID_A);//std::fmod(obj[n].p.ori + th, 2 * M_PI);
		float th = rot * 2 * M_PI / GRID_A;
		for (k = 0; k < o->pts.size(); k++) {
				EnvMap[(int)round(o->pts[k].y)][(int)round(o->pts[k].x)] = EMPTY;
			}
		EnvMap[(int)round(o->p.pos.y)][(int)round(o->p.pos.x)] = EMPTY;

		for (k = 0; k < o->pts.size(); k++) {
			ii = o->pts[k].y - o->p.pos.y;
			jj = o->pts[k].x - o->p.pos.x;
			jj = jj + tan(th/2) * ii;		// 3 shear rotation
			ii = sin(-th) * jj + ii;
			jj = jj + tan(th/2) * ii;
			o->pts[k].y = o->p.pos.y + ii;
			o->pts[k].x = o->p.pos.x + jj;
			//ii = -sin(th) * j + cos(th) * i; 	
			//jj = cos(th) * j + sin(th)	* i;
			//std::cout << "Ori : " << j << "," << i << " Rot : " << jj << "," << ii << std::endl;
		}

		for (k = 0; k < o->pts.size(); k++) {
				// EnvMap[(int)floor(obj[n].pts[i].y)][(int)floor(obj[n].pts[i].x)] = OBJECT;
				// EnvMap[(int)ceil(obj[n].pts[i].y)][(int)ceil(obj[n].pts[i].x)] = OBJECT;
				EnvMap[(int)round(o->pts[k].y)][(int)round(o->pts[k].x)] = OBJECT;
			}
		EnvMap[(int)round(o->p.pos.y)][(int)round(o->p.pos.x)] = OBJCG;
	}

	// ############################################## @$ ##############################################

	void translateObject(object* o, float dx, float dy)
	{
		float k,ii,jj;
		
		for (k = 0; k < o->pts.size(); k++)
			EnvMap[(int)round(o->pts[k].y)][(int)round(o->pts[k].x)] = EMPTY;
		EnvMap[(int)round(o->p.pos.y)][(int)round(o->p.pos.x)] = EMPTY;

		for (k = 0; k < o->pts.size(); k++) {
			o->pts[k].y += dy;
			o->pts[k].x += dx;
		}
		o->p.pos.y += dy;
		o->p.pos.x += dx;		

		for (k = 0; k < o->pts.size(); k++)
			EnvMap[(int)round(o->pts[k].y)][(int)round(o->pts[k].x)] = OBJECT;
		EnvMap[(int)round(o->p.pos.y)][(int)round(o->p.pos.x)] = OBJCG;
	}

	// ############################################## @$ ##############################################

	void buildCSpace(object* o, int th)
	{
		int i,j,k,kk,kkk;
		float dx,dy;
		//for (th = 0; th < GRID_A; th++) {
			// Rotate object
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++) {
					o->cSpace[th][i][j] = true;
					for (k = 0; k < o->pts.size(); k++) {
						dx = j + o->pts[k].x - o->p.pos.x;
						dy = i + o->pts[k].y - o->p.pos.y;
						if (dx < 0 || dy < 0 || dx >= GRID_X || dy >= GRID_Y) {
							o->cSpace[th][i][j] = false;
							break;
						}
						else if (EnvMap[(int)round(dy)][(int)round(dx)] == OBSTACLE)
						{
							o->cSpace[th][i][j] = false;
						} 
						else if (EnvMap[(int)round(dy)][(int)round(dx)] == OBJECT ||  EnvMap[(int)round(dy)][(int)round(dx)] == OBJCG) {
							for (kk = 0; kk < o->pts.size(); kk++) {
								if ((int)round(dy) == (int)round(o->pts[kk].y) && (int)round(dx) == (int)round(o->pts[kk].x))
									break;
							}
							/*for (kkk = 0; kkk < o->dks.size(); kkk++) {
								if ((int)round(dy) == (int)round(o->dks[kkk].pos.y) && (int)round(dx) == (int)round(o->dks[kkk].pos.x))
									break;
							}*/
							if (kk == o->pts.size()) {//} && kkk == o->dks.size()) {
								if ((int)round(dy) == (int)round(o->p.pos.y) && (int)round(dx) == (int)round(o->p.pos.x))
									break;
								o->cSpace[th][i][j] = false;
								break;
							}							
						}
					}
				}
			}
		//}
	}

	// ############################################## @$ ##############################################

	void printCSpace(object* o, int th)
	{
		int i,j,k;
		//std::cout << "Configuration Space for Object " << n << " at angle " << obj[n].p.ori << std::endl;
		//for (th = 0; th < 1; th++) {
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++) {
					if (o->cSpace[th][i][j] == true)
						std::cout << "x ";
					else
						std::cout << ". ";
				}
				std::cout << std::endl;
			}
		//}
	}

	// ############################################## @$ ##############################################

	int distM(int x1, int y1, int x2, int y2)	// Manhattan Distance
	{
		if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0)
			return infVal;
		return std::abs(x1-x2) + std::abs(y1-y2);
	}

	// ############################################## @$ ##############################################

	int distC(int x1, int y1, int x2, int y2)	// Cartesian Distance
	{
		return std::sqrt(std::pow(x1-x2,2) + std::pow(y1-y2,2));
	}

	// ############################################## @$ ##############################################

	void printEnvMap()
	{
		int i,j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (EnvMap[i][j] == EMPTY) {
					std::cout << " .";
				}
				/*else {
					std::cout << EnvMap[i][j] << " ";
				}*/
				else if (EnvMap[i][j] == OBSTACLE) {
					std::cout << "NN";
				}
				else if (EnvMap[i][j] == EXPLORED) {
					std::cout << ", ";
				}
				else if (EnvMap[i][j] == DOCK) {
					std::cout << "{}";
				}
				else if (EnvMap[i][j] == OBJCG) {
					std::cout << "CG";
				}
				else if (EnvMap[i][j] == POC) {
					std::cout << "()";
				}
				else if (EnvMap[i][j] == OBJECT) {
					std::cout << "[]";
				}
				else if (EnvMap[i][j] == AGENT) {
					std::cout << "##";
				}
				else if (EnvMap[i][j] == TARGET) {
					std::cout << "xx";
				}
				else if (EnvMap[i][j] == AGENTCG) {
					std::cout << "CG";
				}
				else if (EnvMap[i][j] == PATH) {
					std::cout << "x ";
				}				
			}
			std::cout << std::endl;
		}
		/*for (i = 0; i < GRID_X	; i++) {
			std::cout << "||";
		}*/
		std::cout << std::endl;
	}

	// ############################################## @$ ##############################################

};

int main(int argc, char** argv)
{
  CagingTestbed ctb;
  ctb.cage();
  return 1; 
}
