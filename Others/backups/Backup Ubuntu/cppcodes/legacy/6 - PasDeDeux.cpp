/**
 * \file PasDeDeux.cpp
 * 
 * Internship Project at TCS Innovation Labs Kolkata
 * 
 * \author Aritra Sarkar
 * \date 08-08-2017 (begin)
 */

/* Constraints :
	-	Object n pixels
	-	Agent 1 pixels
	-	1 Objects
	-	m Agents

	Algorithm :
	1)	Find A* path of object
	2)	If no path exist, end of program (fail)
	3)	For each step, check agent position valid
	4)	If not, continue to step 8
	5)	For each step, check agent path from (t-1) position exist
	6)	If not, continue to step 8
	7)	Display valid path, end of program (success)
	8)	Increase heuristic cost of object location at that step, jump to step 1
 *	
 */

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <unistd.h>		// for usleep
 
const int GRID_X = 20;
const int GRID_Y = 20;

const int GRID_A = 8;	// Rotation resolution is 2*PI/GRID_A
const int NUM_OBJ = 1;
const int NUM_AGT = 2;

const int SIM_TIME = 5;
const int SIM_DELAY = 150000;

bool hCostInfAllow;

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
	TURN_AC	= 5,
	SKIP	= 6
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

class PasDeDeux
{

public:

	bool EnvMap[GRID_Y][GRID_X];	// 0 - free cell, 1 - wall, 2 - agent, 3xx - object
	bool hCostInf[GRID_Y][GRID_X];
	object obj[NUM_OBJ];
	object agt[NUM_AGT];
	std::vector<node> nodeQ;
	int infVal;
	int time;
	
	// ############################################## @$ ##############################################

	PasDeDeux()
	{
		std::cout << "\033[2J\033[1;1H";
		time = 0;
		infVal = 100000;
		int i,j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				EnvMap[i][j] = true;
				hCostInf[i][j] = false;
			}
		}
		initWall();
	}
	
	// ############################################## @$ ##############################################

	void cage()
	{
		int i, j, k, n, agtrr;
		bool solve = false;
		std::vector<int> agttrail[NUM_AGT];
		std::vector<int> objtrail[NUM_OBJ];
		
		for (; time < SIM_TIME; time++) {
			std::cout << "Sim Try " << time << std::endl;
			initAgents();
			initObjects();
			for (n = 0; n < NUM_OBJ; n++)	
				buildCSpace(&obj[n]);
			printDebug();
			hCostInfAllow = true;
			for (n = 0; n < NUM_OBJ; n++)				
				objtrail[n] = pathObject(n);
			for (n = 0; n < NUM_OBJ; n++) {	// For each object
				printTrail(objtrail[n]);
				if (objtrail[n][0] == SKIP) {
					printDebug();
					time = SIM_TIME;
					break;
				}
				for (k = 0; k < objtrail[n].size();k++) {	// For Each Object Movement Frame	
					updateDocks(&obj[n]);
					obj[n].target = obj[n].p;
					switch (objtrail[n][k]) {
						case MOVE_E		:	obj[n].target.pos.x++;	break;
						case MOVE_N		: 	obj[n].target.pos.y--;	break;
						case MOVE_W		: 	obj[n].target.pos.x--;	break;
						case MOVE_S		:	obj[n].target.pos.y++;	break;
						case TURN_C		: 	obj[n].target.ori--;	break;	// WATSON : MODULO
						case TURN_AC	: 	obj[n].target.ori++;	break;
					}
					selectDocks(&obj[n]);
					printDebug();
					std::cout << "\nObject " << n << " ";
					printTrail(objtrail[n]);
					for (agtrr = 0; agtrr < NUM_AGT; agtrr++) {
						if (!checkAssignment(n,agtrr))
							continue;
						buildCSpace(&agt[agtrr]);
						updateEndEfctr(&agt[agtrr]);	// WATSON : Merge functionality with updateDocks(&agt[agtrr]);
						hCostInfAllow = false;
						agttrail[agtrr] = pathAgent(agtrr,n);
						if (agttrail[agtrr].size() == 1 && agttrail[agtrr][0] == SKIP) {
							printDebug();
							std::cout << "\nObject " << n << " ";
							printTrail(objtrail[n]);
							continue;
						}
						else if (agttrail[agtrr].size() != 0) {
							for (i = 0; i < agttrail[agtrr].size(); i++) {
								switch (agttrail[agtrr][i]) {
									case MOVE_E		: translateObject(&agt[agtrr],1,0);		break;
									case MOVE_N		: translateObject(&agt[agtrr],0,-1);	break;
									case MOVE_W		: translateObject(&agt[agtrr],-1,0);	break;
									case MOVE_S		: translateObject(&agt[agtrr],0,1);		break;
									case TURN_C		: rotateObject(&agt[agtrr],-1);			break;
									case TURN_AC	: rotateObject(&agt[agtrr],1);			break;
								}
								printDebug();
								//std::cout << "Agent " << agtrr << " ";
								//printTrail(agttrail[agtrr]);
								std::cout << "\nObject " << n << " ";
								printTrail(objtrail[n]);
							}
						}
						agt[agtrr].pushActive = true;					
					}
					// WATSON : check all rovers in docking position
					if (agt[0].pushActive && agt[1].pushActive) {
						switch (objtrail[n][k]) {
							case MOVE_E		: translateObject(&obj[n],1,0);		break;
							case MOVE_N		: translateObject(&obj[n],0,-1);	break;
							case MOVE_W		: translateObject(&obj[n],-1,0);	break;
							case MOVE_S		: translateObject(&obj[n],0,1);		break;
							case TURN_C		: rotateObject(&obj[n],-1);			break;
							case TURN_AC	: rotateObject(&obj[n],1);			break;
						}
						agt[0].pushActive = false;
						agt[1].pushActive = false;
						if (k == objtrail[n].size()-1) {
							solve = true;
							time = SIM_TIME;
						}
					}
					else {
						std::cout << "No further path for any agents\n";
						hCostInf[(int)round(obj[n].p.pos.y)][(int)round(obj[n].p.pos.x)] = true;
						break;
					}
				}
				obj[n].dks.clear();
				printDebug();
			}				
		}
		if (solve)
			std::cout << "Maze solvable" <<"\n";
		else
			std::cout << "Maze not solvable in " << time << " tries" <<"\n";										
	}

	// ############################################## @$ ##############################################

private:
	
	void initWall()
	{
		int k;
		for (k = 0; k < GRID_X; k++) {
			EnvMap[0][k] = false;
			EnvMap[GRID_Y-1][k] = false;
		}
		for (k = 0; k < GRID_Y; k++) {
			EnvMap[k][0] = false;
			EnvMap[k][GRID_X-1] = false;
		}
		for (k = 2; k < GRID_X-2; k++) {
			EnvMap[8][k] = false;
		}
		EnvMap[8][7] = true;
		EnvMap[8][8] = true;
		EnvMap[8][9] = true;
	}

	// ############################################## @$ ##############################################
	
	void initAgents()
	{
		point z;
		
		z.x = 1;
		z.y = 1;
		agt[0].pts.clear();
		agt[0].dks.clear();
		agt[0].pts.push_back(z);
		agt[0].p.pos.x = 1;
		agt[0].p.pos.y = 1;
		agt[0].p.ori = 6;
		agt[0].wgt = 1;
		agt[0].pushActive = false;

		z.x = 18;
		z.y = 1;
		agt[1].pts.clear();
		agt[1].dks.clear();
		agt[1].pts.push_back(z);
		agt[1].p.pos.x = 18;
		agt[1].p.pos.y = 1;
		agt[1].p.ori = 4;
		agt[1].wgt = 1;
		agt[1].pushActive = false;
	}

	// ############################################## @$ ##############################################
	
	void initObjects()
	{
		int i, j;
		point z;
		obj[0].pts.clear();
		obj[0].dks.clear();

		for (i = 13; i < 13+3; i++) {
			for (j = 2; j < 2+5; j++) {
				z.y = i;
				z.x = j;
				obj[0].pts.push_back(z);
			}
		}		
		obj[0].p.pos.x = 4;
		obj[0].p.pos.y = 14;
		obj[0].p.ori = 0;
		obj[0].wgt = 2;
		obj[0].pushActive = false;
		obj[0].target.pos.x = 13;
		obj[0].target.pos.y = 2;
		obj[0].target.ori = 0;
	}

	// ############################################## @$ ##############################################
	
	void printDebug()
	{
		int n;
		printWorld();
		for (n = 0; n < NUM_OBJ; n++)	
			printObject(n,false);
		for (n = 0; n < NUM_AGT; n++)	
			printObject(n,true);
		
		//printEnvMap();
	}

	// ############################################## @$ ##############################################
	
	bool checkAssignment(int agtId, int objId)
	{
		// WATSON : Add agent allocation algorithm here
		return true;
	}

	// ############################################## @$ ##############################################
	
	std::vector<int> pathAgent(int agtId, int objId)
	{
		int i, j, th, d, iv;
		pose p1,p2;	// agt_cg, obj_cg
		std::vector<int> trail;
		bool cSpaceTmp[GRID_A][GRID_Y][GRID_X];
		
		p1.pos.x = (int)round(agt[agtId].p.pos.x);
		p1.pos.y = (int)round(agt[agtId].p.pos.y);
		p1.ori = (agt[agtId].p.ori);
		for (th = 0; th < GRID_A; th++) {
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++)
					cSpaceTmp[th][i][j] = agt[agtId].cSpace[th][i][j];
			}
		}		
		for (d = 0; d < obj[objId].dks.size(); d++) {
			nodeQ.clear();
			trail.clear();
			iv = infVal;
			p2.pos.x = (int)round(obj[objId].dks[d].pos.x);
			p2.pos.y = (int)round(obj[objId].dks[d].pos.y);
			p2.ori = (obj[objId].dks[d].ori);
			if(checkGoal(&agt[agtId],p1,p2,p1))
				return trail;
			NextMove(&agt[agtId], p1, p2, 0, trail);
			while (nodeQ.size() > 0) {
				node nn = nodeQ.front();
				if(nn.tCost < iv && checkGoal(&agt[agtId],p1,p2,nn.p)) {
					trail = nn.trail;
					iv = nn.tCost;
					break;
				}
				nodeQ.erase(nodeQ.begin());
				NextMove(&agt[agtId], nn.p, p2, nn.gCost, nn.trail);
			}
			for (th = 0; th < GRID_A; th++) {
				for (i = 0; i < GRID_Y; i++) {
					for (j = 0; j < GRID_X; j++)
						agt[agtId].cSpace[th][i][j] = cSpaceTmp[th][i][j];
				}
			}
			if (trail.size() != 0)
				break;
		}
		if (trail.size() == 0)
			trail.push_back(SKIP);
		return trail;
	}

	// ############################################## @$ ##############################################
	
	bool checkGoal(object* o, pose p1, pose p2, pose nnp)
	{
		int j;
		for (j = 0; j < o->dks.size(); j++) {
			switch ((int)round(o->dks[j].ori)) {
				case 0			:
					if (nnp.pos.x-p1.pos.x+o->dks[j].pos.x-1 == p2.pos.x &&
						nnp.pos.y-p1.pos.y+o->dks[j].pos.y == p2.pos.y &&
						p2.ori == GRID_A/2){
						return true;
					}
				case GRID_A/4	:
					if (nnp.pos.x-p1.pos.x+o->dks[j].pos.x == p2.pos.x &&
						nnp.pos.y-p1.pos.y+o->dks[j].pos.y+1 == p2.pos.y &&
						p2.ori == 3*GRID_A/4)
						return true;
				case GRID_A/2	:
					if (nnp.pos.x-p1.pos.x+o->dks[j].pos.x+1 == p2.pos.x &&
						nnp.pos.y-p1.pos.y+o->dks[j].pos.y == p2.pos.y &&
						p2.ori == 0)
						return true;
				case 3*GRID_A/4	:
					if (nnp.pos.x-p1.pos.x+o->dks[j].pos.x == p2.pos.x &&
						nnp.pos.y-p1.pos.y+o->dks[j].pos.y-1 == p2.pos.y &&
						p2.ori == GRID_A/4)
						return true;
			}
		}
		return false;
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
			tCost = travelled + CMOVE_W + distM(p2.pos.x,p2.pos.y,x-1,y);// + std::abs(p2.ori-th);
			o->cSpace[th][y][x-1] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x-1,y,th,travelled+CMOVE_W,tCost,trail,MOVE_W);
		}

		if (x < GRID_X-1 && o->cSpace[th][y][x+1]) {
			tCost = travelled + CMOVE_E + distM(p2.pos.x,p2.pos.y,x+1,y);// + std::abs(p2.ori-th);
			o->cSpace[th][y][x+1] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x+1,y,th,travelled+CMOVE_E,tCost,trail,MOVE_E);
		}

		if (y > 0 && o->cSpace[th][y-1][x]) {
			tCost = travelled + CMOVE_N + distM(p2.pos.x,p2.pos.y,x,y-1);// + std::abs(p2.ori-th);
			o->cSpace[th][y-1][x] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x,y-1,th,travelled+CMOVE_N,tCost,trail,MOVE_N);
		}

		if (y < GRID_Y-1 && o->cSpace[th][y+1][x]) {
			tCost = travelled + CMOVE_S + distM(p2.pos.x,p2.pos.y,x,y+1);// + std::abs(p2.ori-th);
			o->cSpace[th][y+1][x] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x,y+1,th,travelled+CMOVE_S,tCost,trail,MOVE_S);
		}

		if (o->cSpace[(int)std::fmod(th-1+GRID_A,GRID_A)][y][x]) {
			tCost = travelled + CTURN_C + distM(p2.pos.x,p2.pos.y,x,y);// + std::abs(p2.ori-(th-1));
			o->cSpace[(int)std::fmod(th-1+GRID_A,GRID_A)][y][x] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x,y,(int)std::fmod(th-1+GRID_A,GRID_A),travelled+CTURN_C,tCost,trail,TURN_C);
		}

		if (o->cSpace[(int)std::fmod(th+1,GRID_A)][y][x]) {
			tCost = travelled + CTURN_AC + distM(p2.pos.x,p2.pos.y,x,y);// + std::abs(p2.ori-(th+1));
			o->cSpace[(int)std::fmod(th+1,GRID_A)][y][x] = false;	// better explored marking mechanism
			if (tCost < infVal)
				EnqueNode(x,y,(int)std::fmod(th+1,GRID_A),travelled+CTURN_AC,tCost,trail,TURN_AC);
		}
	}

	// ############################################## @$ ##############################################

	std::vector<int> pathObject(int objId)
	{
		std::vector<int> trail;
		nodeQ.clear();
		int iv = infVal;
		pose p1, p2;	// cg, target
		p1.pos.x = (int)round(obj[objId].p.pos.x);
		p1.pos.y = (int)round(obj[objId].p.pos.y);
		p1.ori = (obj[objId].p.ori);//(int)round
		p2.pos.x = (int)round(obj[objId].target.pos.x);
		p2.pos.y = (int)round(obj[objId].target.pos.y);
		p2.ori = (int)round(obj[objId].target.ori);
		NextMove(&obj[objId], p1, p2, 0, trail);
		while (nodeQ.size() > 0) {
			node nn = nodeQ.front();
			if(nn.p.pos.x == p2.pos.x && nn.p.pos.y == p2.pos.y && nn.p.ori == p2.ori  && nn.tCost < iv) {
				trail = nn.trail;
				iv = nn.tCost;
			}
			nodeQ.erase(nodeQ.begin());
			NextMove(&obj[objId], nn.p, p2, nn.gCost, nn.trail);
		}
		if (trail.size() == 0)
			trail.push_back(SKIP);
		return trail;
	}

	// ############################################## @$ ##############################################

	void updateEndEfctr(object* o)
	{
		int i, j, k;
		pose d;
		o->dks.clear();
		for (k = 0; k < o->pts.size(); k++) {
			i = round(o->pts[k].y);
			j = round(o->pts[k].x);
			d.pos.y = i;
			d.pos.x = j;
			if (i-1 >= 0) {
				d.ori = 3*GRID_A/4;						// rover should face down to dock/push
				o->dks.push_back(d);
			}
			if (i+1 < GRID_Y) {
				d.ori = GRID_A/4;
				o->dks.push_back(d);
			}
			if (j-1 >= 0) {
				d.ori = 0;
				o->dks.push_back(d);
			}
			if (j+1 < GRID_X) {
				d.ori = GRID_A/2;
				o->dks.push_back(d);
			}
		}
	}

	// ############################################## @$ ##############################################

	void updateDocks(object* o)
	{
		int i, j, k, n;
		pose d;
		bool World[GRID_Y][GRID_X];
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++)
				World[i][j] = EnvMap[i][j];				
		}
		for (n = 0; n < NUM_OBJ; n++) {
			for (i = 0; i < obj[n].pts.size(); i++)
				World[(int)round(obj[n].pts[i].y)][(int)round(obj[n].pts[i].x)] = false;
		}
		/*for (n = 0; n < NUM_AGT; n++) {
			for (i = 0; i < agt[n].pts.size(); i++)
				World[(int)round(agt[n].pts[i].y)][(int)round(agt[n].pts[i].x)] = false;
		}*/
		o->dks.clear();
		for (k = 0; k < o->pts.size(); k++) {
			i = round(o->pts[k].y);
			j = round(o->pts[k].x);
			d.pos.y = i;
			d.pos.x = j;
			if (i-1 >= 0 && World[i-1][j]) {
				d.ori = 3*GRID_A/4;						// rover should face down to dock/push
				o->dks.push_back(d);
			}
			if (i+1 < GRID_Y && World[i+1][j]) {
				d.ori = GRID_A/4;
				o->dks.push_back(d);
			}
			if (j-1 >= 0 && World[i][j-1]) {
				d.ori = 0;
				o->dks.push_back(d);
			}
			if (j+1 < GRID_X && World[i][j+1]) {
				d.ori = GRID_A/2;
				o->dks.push_back(d);
			}
		}
	}

	// ############################################## @$ ##############################################

	void selectDocks(object* o)
	{
		int k;
		for (k = 0; k < o->dks.size(); k++) {
			if (o->p.pos.x == o->target.pos.x && o->p.pos.y == o->target.pos.y) {
				if (o->dks[k].pos.x < o->p.pos.x && o->dks[k].pos.y < o->p.pos.y) {
					if ((o->target.ori - o->p.ori) > 0 && o->dks[k].ori == 3*GRID_A/4 ||
						(o->target.ori - o->p.ori) < 0 && o->dks[k].ori == 0)
						continue;
				}
				else if (o->dks[k].pos.x < o->p.pos.x && o->dks[k].pos.y > o->p.pos.y) {
					if ((o->target.ori - o->p.ori) > 0 && o->dks[k].ori == 0 ||
						(o->target.ori - o->p.ori) < 0 && o->dks[k].ori == GRID_A/4)
						continue;
				}
				else if (o->dks[k].pos.x > o->p.pos.x && o->dks[k].pos.y < o->p.pos.y) {
					if ((o->target.ori - o->p.ori) > 0 && o->dks[k].ori == GRID_A/2 ||
						(o->target.ori - o->p.ori) < 0 && o->dks[k].ori == 3*GRID_A/4)
						continue;
				}
				else if (o->dks[k].pos.x > o->p.pos.x && o->dks[k].pos.y > o->p.pos.y) {
					if ((o->target.ori - o->p.ori) > 0 && o->dks[k].ori == GRID_A/4 ||
						(o->target.ori - o->p.ori) < 0 && o->dks[k].ori == GRID_A/2)
						continue;
				}
			}
			else {
				if (o->dks[k].ori == 0 && 
					distM(o->p.pos.x+1,o->p.pos.y,o->target.pos.x,o->target.pos.y) < distM(o->p.pos.x,o->p.pos.y,o->target.pos.x,o->target.pos.y))
					continue;
				else if (o->dks[k].ori == GRID_A/4 && 
					distM(o->p.pos.x,o->p.pos.y-1,o->target.pos.x,o->target.pos.y) < distM(o->p.pos.x,o->p.pos.y,o->target.pos.x,o->target.pos.y))
					continue;
				else if (o->dks[k].ori == GRID_A/2 && 
					distM(o->p.pos.x-1,o->p.pos.y,o->target.pos.x,o->target.pos.y) < distM(o->p.pos.x,o->p.pos.y,o->target.pos.x,o->target.pos.y))
					continue;
				else if (o->dks[k].ori == 3*GRID_A/4 && 
					distM(o->p.pos.x,o->p.pos.y+1,o->target.pos.x,o->target.pos.y) < distM(o->p.pos.x,o->p.pos.y,o->target.pos.x,o->target.pos.y))
					continue;
			}
			o->dks.erase(o->dks.begin()+k);
			k--;			
		}
	}

	// ############################################## @$ ##############################################

	void filterDocks(int objId)
	{	
		// keep best n docks, where n = obj.weight
		
	}

	// ############################################## @$ ##############################################

	void buildCSpace(object* o)
	{
		int i, j, deg, th, k, n;
		float dx,dy;
		bool World[GRID_Y][GRID_X];
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++)
				World[i][j] = EnvMap[i][j];				
		}
		for (n = 0; n < NUM_OBJ; n++) {
			for (i = 0; i < obj[n].pts.size(); i++)
				World[(int)round(obj[n].pts[i].y)][(int)round(obj[n].pts[i].x)] = false;
			for (i = 0; i < obj[n].dks.size(); i++)
				World[(int)round(obj[n].dks[i].pos.y)][(int)round(obj[n].dks[i].pos.x)] = false;
		}
		for (n = 0; n < NUM_AGT; n++) {
			for (i = 0; i < agt[n].pts.size(); i++)
				World[(int)round(agt[n].pts[i].y)][(int)round(agt[n].pts[i].x)] = false;
			for (i = 0; i < agt[n].dks.size(); i++)
				World[(int)round(agt[n].dks[i].pos.y)][(int)round(agt[n].dks[i].pos.x)] = false;
		}
		for (i = 0; i < o->pts.size(); i++)
			World[(int)round(o->pts[i].y)][(int)round(o->pts[i].x)] = true;
		for (i = 0; i < o->dks.size(); i++)
			World[(int)round(o->dks[i].pos.y)][(int)round(o->dks[i].pos.x)] = true;
		for (deg = 0; deg < GRID_A; deg++) {
			th = o->p.ori;
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++) {
					o->cSpace[th][i][j] = true;
					for (k = 0; k < o->pts.size(); k++) {
						dx = j - o->p.pos.x + o->pts[k].x;
						dy = i - o->p.pos.y + o->pts[k].y; 
						if (dx < 0 || dy < 0 || dx >= GRID_X || dy >= GRID_Y) {
							o->cSpace[th][i][j] = false;
							break;
						}
						else if (World[(int)round(dy)][(int)round(dx)] == false) {
							o->cSpace[th][i][j] = false;
							break;
						}
					}
				}
			}
			rotateObject(o,1);
		}
	}

	// ############################################## @$ ##############################################

	int distM(int x1, int y1, int x2, int y2)	// Manhattan Distance
	{
		if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0)
			return infVal;
		if (hCostInf[y2][x2] && hCostInfAllow)
			return infVal;
		return std::abs(x1-x2) + std::abs(y1-y2);
	}

	// ############################################## @$ ##############################################

	int distC(int x1, int y1, int x2, int y2)	// Cartesian Distance
	{
		return std::sqrt(std::pow(x1-x2,2) + std::pow(y1-y2,2));
	}

	// ############################################## @$ ##############################################

	void rotateObject(object* o, int rot)
	{
		float i, j, k, th = rot * 2 * M_PI / GRID_A;
		for (k = 0; k < o->pts.size(); k++) {
			i = o->pts[k].y - o->p.pos.y;
			j = o->pts[k].x - o->p.pos.x;
			j = j + tan(th/2) * i;			// 3 shear rotation
			i = sin(-th) * j + i;
			j = j + tan(th/2) * i;
			o->pts[k].y = o->p.pos.y + i;
			o->pts[k].x = o->p.pos.x + j;
		}
		o->p.ori = std::fmod(o->p.ori+rot,GRID_A);
	}

	// ############################################## @$ ##############################################

	void translateObject(object* o, float dx, float dy)
	{
		float i, j, k;
		for (k = 0; k < o->pts.size(); k++) {
			o->pts[k].y += dy;
			o->pts[k].x += dx;
		}
		o->p.pos.y += dy;
		o->p.pos.x += dx;
	}

	// ############################################## @$ ##############################################

	void printObject(int n, bool agent)
	{
		if (agent) {
			std::cout <<  "Info. --> Agent " << n << std::endl;
			std::cout <<  " * Pose : x = " << agt[n].p.pos.x << ", y = " << agt[n].p.pos.y << ", th = " << agt[n].p.ori << std::endl;
			std::cout <<  " * Points(docks) : " << agt[n].pts.size() <<  "(" << agt[n].dks.size() << ")" << std::endl;
			std::cout <<  " * Weight : " << agt[n].wgt << std::endl;
			std::cout <<  " * Active : " << agt[0].pushActive << std::endl;
		}
		else {
			std::cout <<  "Info. --> Object " << n << std::endl;
			std::cout <<  " * Pose : x = " << obj[n].p.pos.x << ", y = " << obj[n].p.pos.y << ", th = " << obj[n].p.ori << std::endl;
			std::cout <<  " * Points(docks) : " << obj[n].pts.size() <<  "(" << obj[n].dks.size() << ")" << std::endl;
			std::cout <<  " * Weight : " << obj[n].wgt << std::endl;
			std::cout <<  " * Active : " << obj[0].pushActive << std::endl;
			std::cout <<  " * Target Pose : x = " << obj[n].target.pos.x << ", y = " << obj[n].target.pos.y <<  ", th = " << obj[n].target.ori << std::endl;
		}
		std::cout << std::endl;	
	}

	// ############################################## @$ ##############################################

	void printCSpace(object* o, int th)
	{
		int i, j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (o->cSpace[th][i][j])
					std::cout << " v";
				else
					std::cout << " x";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}

	// ############################################## @$ ##############################################
	
	void printTrail(std::vector<int> trail)
	{
		int n;
		std::cout << "Planned Moves :";
		for (n = 0; n < trail.size(); n++) {
			if (trail[n] == MOVE_W)
				std::cout << " (" << (n+1) << ",W)";
			if (trail[n] == MOVE_E)
				std::cout << " (" << (n+1) << ",E)";
			if (trail[n] == MOVE_N)
				std::cout << " (" << (n+1) << ",N)";
			if (trail[n] == MOVE_S)
				std::cout << " (" << (n+1) << ",S)";
			if (trail[n] == TURN_C)
				std::cout << " (" << (n+1) << ",C)";
			if (trail[n] == TURN_AC)
				std::cout << " (" << (n+1) << ",AC)";
			if (trail[n] == SKIP)
				std::cout << " (" << (n+1) << ",SKIP)";
			}
		std::cout << std::endl;
	}

	// ############################################## @$ ##############################################

	void printEnvMap()
	{
		int i, j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (EnvMap[i][j])
					std::cout << " .";
				else
					std::cout << "NN";				
			}
			std::cout << std::endl;
		}
	}

	// ############################################## @$ ##############################################

	void printWorld()
	{
		int i, j, n, World[GRID_Y][GRID_X];
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (EnvMap[i][j])
					World[i][j] = EMPTY;
				else
					World[i][j] = OBSTACLE;				
			}
		}
		for (n = 0; n < NUM_OBJ; n++) {
			World[(int)round(obj[n].target.pos.y)][(int)round(obj[n].target.pos.x)] = TARGET;
			for (i = 0; i < obj[n].pts.size(); i++) {
				World[(int)round(obj[n].pts[i].y)][(int)round(obj[n].pts[i].x)] = OBJECT;
			}
			for (i = 0; i < obj[n].dks.size(); i++) {
				World[(int)round(obj[n].dks[i].pos.y)][(int)round(obj[n].dks[i].pos.x)] = DOCK;
			}
			World[(int)round(obj[n].p.pos.y)][(int)round(obj[n].p.pos.x)] = OBJCG;
		}
		for (n = 0; n < NUM_AGT; n++) {
			for (i = 0; i < agt[n].pts.size(); i++) {
				World[(int)round(agt[n].pts[i].y)][(int)round(agt[n].pts[i].x)] = AGENT;
			}
			for (i = 0; i < agt[n].dks.size(); i++) {
				//World[(int)round(agt[n].dks[i].pos.y)][(int)round(agt[n].dks[i].pos.x)] = DOCK;
			}
			//World[(int)round(agt[n].p.pos.y)][(int)round(agt[n].p.pos.x)] = OBJCG;
		}
		usleep(SIM_DELAY);
		std::cout << "\033[2J\033[1;1H";
		std::cout << "Simulation Try " << time << std::endl;
		std::cout << std::endl;
		for (i = 0; i < GRID_Y; i++) {
			std::cout << " ";
			for (j = 0; j < GRID_X; j++) {
				switch (World[i][j]) {
					case EMPTY		:	std::cout << " ."; break;
					case OBSTACLE	:	std::cout << "NN"; break;
					case OBJCG		:	std::cout << "CG"; break;
					case OBJECT		:	std::cout << "[]"; break;
					case AGENT		:	std::cout << "{}"; break;
					case DOCK		:	std::cout << "()"; break;
					case TARGET		:	std::cout << " x"; break;
					case POC		:	std::cout << " O"; break;
					default			:	std::cout << World[i][j];
				}				
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}

	// ############################################## @$ ##############################################
};

int main(int argc, char** argv)
{
	PasDeDeux pdd;
	pdd.cage();
	return 1; 
}