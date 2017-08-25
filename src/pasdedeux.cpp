/**
 * \file pasdedeux.cpp
 * \legacy '9 - PasDeDeuxSense.cpp'
 * 
 * \author Aritra Sarkar
 * \date 16-08-2017 (begin)
 */

#include "pasdedeux.h"

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

class PasDeDeux
{

public:

	bool leader;
	bool EnvMap[GRID_Y][GRID_X];
	bool hCostInf[GRID_Y][GRID_X];
	object obj[NUM_OBJ];
	object agt[NUM_AGT];
	std::vector<node> nodeQ;
	int iv;
	int tick;
	bool hCostInfAllow;
	geometry_msgs::Twist msg;
	ros::Publisher pb[NUM_AGT];
	//ros::Subscriber sb[NUM_AGT];
	ros::NodeHandle nh;	
	//Agent agt[NUM_AGT];
	bool paintWall;
		

	// ############################################## @$ ##############################################

	PasDeDeux(int argc, char** argv)
	{
		srand(time(0));
		leader = electLeader();
		if (!leader)
			return;
		tick = 0;
		int i, j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				EnvMap[i][j] = true;
				hCostInf[i][j] = false;
			}
		}
		initWall();
		std::cout << "Number of Objects simulated = " << NUM_OBJ << std::endl;
		std::cout << "Number of Agents simulated = " << NUM_AGT << std::endl;		
		std::ostringstream oss;
		for(i = 0; i < NUM_AGT; i++) {
			oss.str("");
			oss << "agt" << i << "/cmd_vel";
			pb[i] = nh.advertise<geometry_msgs::Twist>(oss.str(),1000);
			pb[i].publish(msg);	// First msg gets missed
		}
	}	

	// ############################################## @$ ##############################################

	void Adagio()
	{
		int i, j, k, n, agtrr;
		bool solve = false;		
		pose objTgt, err;
		std::vector<int> agttrail[NUM_AGT];
		std::vector<int> objtrail[NUM_OBJ];
		std::vector<int> objtrailBkup, objtrailNew;
		
		initObjects();
		initAgents();
		printDebug();

		assign();
		for (n = 0; n < NUM_OBJ; n++)	
			buildCSpace(&obj[n]);
		hCostInfAllow = true;
		for (n = 0; n < NUM_OBJ; n++)				
			objtrail[n] = pathObject(n);
		hCostInfAllow = false;
		for (n = 0; n < NUM_OBJ; n++) {
			if (objtrail[n][0] == SKIP)
				break;
			objTgt = obj[n].target;
			for (k = 0; k < objtrail[n].size();k++) {	// For Each Object Movement Frame	
				updateDocks(&obj[n]);
				obj[n].target = obj[n].p;
				switch (objtrail[n][k]) {
					case MOVE_E		:	obj[n].target.pos.x++;	break;
					case MOVE_N		: 	obj[n].target.pos.y--;	break;
					case MOVE_W		: 	obj[n].target.pos.x--;	break;
					case MOVE_S		:	obj[n].target.pos.y++;	break;
					case TURN_C		: 	obj[n].target.ori = std::fmod(obj[n].target.ori-1+GRID_A,GRID_A);	break;
					case TURN_AC	: 	obj[n].target.ori = std::fmod(obj[n].target.ori+1,GRID_A);	break;
				}
				selectDocks(&obj[n]);
				filterDocks(n,objtrail[n][k]);
				printDebug();
				for (agtrr = 0; agtrr < NUM_AGT; agtrr++) {
					if (!checkAssignment(agtrr,n))
						continue;
					buildCSpace(&agt[agtrr]);
					updateEndEfctr(&agt[agtrr]);	// WATSON : Merge functionality with updateDocks(&agt[agtrr]);
					agttrail[agtrr] = pathAgent(agtrr,n);
					if (agttrail[agtrr].size() == 1 && agttrail[agtrr][0] == SKIP) {
						continue;
					}
					else if (agttrail[agtrr].size() != 0) {
						// Issue Actuation Command to Hardware here ...
						//agt[i].navigate(i,msg,GridWorld);
						//pb[i].publish(msg);
						//ROS_INFO_STREAM("Cmd Sent"<<" Linear="<<msg.linear.x<<" Angular="<<msg.angular.z << " Debug" << floor(4*double(rand())/double(RAND_MAX)));
			

						err = estimateErr(&agt[agtrr]);
						switch (agttrail[agtrr][0]) {
							case MOVE_E		: translateObject(&agt[agtrr],actuateT(senseWhlEnc(1-err.pos.x)),0);	break;
							case MOVE_N		: translateObject(&agt[agtrr],0,-actuateT(senseWhlEnc(1-err.pos.y)));	break;
							case MOVE_W		: translateObject(&agt[agtrr],-actuateT(senseWhlEnc(1-err.pos.x)),0);	break;
							case MOVE_S		: translateObject(&agt[agtrr],0,actuateT(senseWhlEnc(1-err.pos.y)));	break;
							case TURN_C		: rotateObject(&agt[agtrr],actuateR(-senseWhlEnc(1-err.ori)));			break;
							case TURN_AC	: rotateObject(&agt[agtrr],actuateR(senseWhlEnc(1-err.ori)));			break;
						}
						printDebug();
						std::cout << "\nObject " << n << " ";
						printTrail(objtrail[n]);	//printTrail(agttrail[agtrr]);
						agtrr--;
						continue;
					}
					agt[agtrr].pushActive = true;										
				}
				if (dockingComplete(n)) {
					switch (objtrail[n][k]) {
						case MOVE_E		: translateObject(&obj[n],1,0);		break;
						case MOVE_N		: translateObject(&obj[n],0,-1);	break;
						case MOVE_W		: translateObject(&obj[n],-1,0);	break;
						case MOVE_S		: translateObject(&obj[n],0,1);		break;
						case TURN_C		: rotateObject(&obj[n],-1);			break;
						case TURN_AC	: rotateObject(&obj[n],1);			break;
					}
					for (i = 0; i < NUM_AGT; i++) {
						if (checkAssignment(i,n))
							agt[i].pushActive = false;
					}
					if (k == objtrail[n].size()-1)
						solve = true;
				}
				else {
					hCostInf[(int)round(obj[n].p.pos.y)][(int)round(obj[n].p.pos.x)] = true;
					objtrailBkup = objtrail[n];
					obj[n].target = objTgt;	
					hCostInfAllow = true;
					do {	// handle k = 0
						switch (objtrailBkup[--k]) {
							case MOVE_E		: translateObject(&obj[n],-1,0);	break;
							case MOVE_N		: translateObject(&obj[n],0,1);		break;
							case MOVE_W		: translateObject(&obj[n],1,0);		break;
							case MOVE_S		: translateObject(&obj[n],0,-1);	break;
							case TURN_C		: rotateObject(&obj[n],1);			break;
							case TURN_AC	: rotateObject(&obj[n],-1);			break;
						}
						objtrail[n] = pathObject(n);
					} while (objtrail[n][0] == SKIP && k > 0);
					hCostInfAllow = false;
					objtrailNew.clear();
					for (i = 0; i < k; i++)
						objtrailNew.push_back(objtrailBkup[i]);
					for (i = 0; i < objtrail[n].size(); i++)
						objtrailNew.push_back(objtrail[n][i]);
					objtrail[n] = objtrailNew;
					k--;
					printDebug();
					std::cout << "\nObject " << n << " ";
					printTrail(objtrail[n]);
					std::cout << "No further path for any agents, backtracking\n";
					continue;
				}
			}
			obj[n].dks.clear();
		}
		printDebug();
		if (solve)
			std::cout << "Maze solvable in " << (tick+1) << " tries" <<"\n";
		else
			std::cout << "Maze not solvable in " << (tick+1) << " tries" <<"\n";
	}

	// ############################################## @$ ##############################################

private:
	bool electLeader()
	{
		return true;
	}

	// ############################################## @$ ##############################################

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
		for (k = 2; k < GRID_X; k++) {
			EnvMap[8][k] = false;
		}
		EnvMap[8][7] = true;
		EnvMap[8][8] = true;
		EnvMap[8][9] = true;
		EnvMap[8][GRID_X-3] = true;
		EnvMap[8][GRID_X-4] = true;
		EnvMap[8][GRID_X-5] = true;
		EnvMap[4][7] = false;
		paintWall = true;
	}

	// ############################################## @$ ##############################################
	
	void initObjects()
	{
		int i, j, o, szx = 5, szy = 3;
		float x, y, th;
		point z;
		for (o = 0; o < NUM_OBJ; o++) {
			obj[o].pts.clear();
			obj[o].dks.clear();
			for (i = 0; i < szy; i++) {
				for (j = 0; j < szx; j++) {
					z.y = i;
					z.x = j;
					obj[o].pts.push_back(z);
				}
			}
			obj[o].p.pos.x = 0;
			obj[o].p.pos.y = 0;
			for (i = 0; i < obj[o].pts.size(); i++) {
				obj[o].p.pos.x += obj[o].pts[i].x;
				obj[o].p.pos.y += obj[o].pts[i].y;
			}
			obj[o].p.pos.x /= obj[o].pts.size();
			obj[o].p.pos.y /= obj[o].pts.size();
			obj[o].p.ori = 0;
			buildCSpace(&obj[o]);
			do {
				y = (int)round(GRID_Y * double(rand()) / double(RAND_MAX));
				x = (int)round(GRID_X * double(rand()) / double(RAND_MAX));
				th = 0;//(int)floor(GRID_A * double(rand()) / double(RAND_MAX));
			} while(!obj[o].cSpace[(int)round(th)][(int)round(y)][(int)round(x)]);
			translateObject(&obj[o],x-obj[o].p.pos.x,y-obj[o].p.pos.y);
			rotateObject(&obj[o],th);
			obj[o].wgt = -2;
			obj[o].pushActive = false;
			obj[o].target.pos.x = 13;
			obj[o].target.pos.y = 2;
			obj[o].target.ori = 0;
		}		
	}

	// ############################################## @$ ##############################################
	
	void initAgents()
	{
		int i, a;
		point z;
		for (a = 0; a < NUM_AGT; a++) {		
			agt[a].pts.clear();
			do {
				z.y = GRID_Y * double(rand()) / double(RAND_MAX);
				z.x = GRID_X * double(rand()) / double(RAND_MAX);
			} while(!EnvMap[(int)round(z.y)][(int)round(z.x)]);
			agt[a].pts.push_back(z);

			agt[a].p.ori = GRID_A * double(rand())/double(RAND_MAX);
			agt[a].p.pos.y = 0;
			agt[a].p.pos.x = 0;
			for (i = 0; i < agt[a].pts.size(); i++) {
				agt[a].p.pos.y += agt[a].pts[i].y;
				agt[a].p.pos.x += agt[a].pts[i].x;
			}
			agt[a].p.pos.y /= agt[a].pts.size();
			agt[a].p.pos.x /= agt[a].pts.size();
			
			agt[a].dks.clear();
			agt[a].wgt = 1;
			agt[a].pushActive = false;
			agt[a].asgnmnt = INFVAL;
		}
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
		}
		for (n = 0; n < NUM_AGT; n++) {
			for (i = 0; i < agt[n].pts.size(); i++)
				World[(int)round(agt[n].pts[i].y)][(int)round(agt[n].pts[i].x)] = false;
		}
		for (i = 0; i < o->pts.size(); i++)
			World[(int)round(o->pts[i].y)][(int)round(o->pts[i].x)] = true && EnvMap[(int)round(o->pts[i].y)][(int)round(o->pts[i].x)];
		for (i = 0; i < o->dks.size(); i++)
			World[(int)round(o->dks[i].pos.y)][(int)round(o->dks[i].pos.x)] = true && EnvMap[(int)round(o->dks[i].pos.y)][(int)round(o->dks[i].pos.x)];
		for (deg = 0; deg < GRID_A; deg++) {
			th = (int)floor(o->p.ori);
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
		o->p.ori = std::fmod(o->p.ori+rot+GRID_A,GRID_A);
	}

	// ############################################## @$ ##############################################
	
	void assign()
	{
		// WATSON : Add agent allocation algorithm here
		// E.g. - Hungarian Algorithm
		// currently, FCFS
		int o, a;
		for (o = 0; o < NUM_OBJ; o++) {
			for (a = 0; a < NUM_AGT; a++) {
				if (obj[o].wgt != 0 && agt[a].wgt != 0) {
					obj[o].wgt++;
					agt[a].wgt--;
					agt[a].asgnmnt = o;
				}
			}
		}
	}

	// ############################################## @$ ##############################################
	
	bool checkAssignment(int agtId, int objId)
	{
		if (agt[agtId].asgnmnt == objId)
			return true;
		return false;
	}

	// ############################################## @$ ##############################################
	
	bool dockingComplete(int objId)
	{
		for (int a = 0; a < NUM_AGT; a++) {
			if (agt[a].asgnmnt == objId && agt[a].pushActive == false)
				return false;
		}
		return true;
	}

	// ############################################## @$ ##############################################
	
	bool checkGoal(object* o, pose p1, pose p2, pose nnp)
	{
		int j;
		for (j = 0; j < o->dks.size(); j++) {
			switch ((int)round(nnp.ori)) {
				case 0			:
					if (nnp.pos.x-p1.pos.x+o->dks[j].pos.x-1 == p2.pos.x &&
						nnp.pos.y-p1.pos.y+o->dks[j].pos.y == p2.pos.y &&
						p2.ori == GRID_A/2)
						return true;
					break;
				case GRID_A/4	:
					if (nnp.pos.x-p1.pos.x+o->dks[j].pos.x == p2.pos.x &&
						nnp.pos.y-p1.pos.y+o->dks[j].pos.y+1 == p2.pos.y &&
						p2.ori == 3*GRID_A/4)
						return true;
					break;
				case GRID_A/2	:
					if (nnp.pos.x-p1.pos.x+o->dks[j].pos.x+1 == p2.pos.x &&
						nnp.pos.y-p1.pos.y+o->dks[j].pos.y == p2.pos.y &&
						p2.ori == 0)
						return true;
					break;
				case 3*GRID_A/4	:
					if (nnp.pos.x-p1.pos.x+o->dks[j].pos.x == p2.pos.x &&
						nnp.pos.y-p1.pos.y+o->dks[j].pos.y-1 == p2.pos.y &&
						p2.ori == GRID_A/4)
						return true;
					break;
			}
		}
		return false;
	}

	// ############################################## @$ ##############################################
	
	std::vector<int> pathAgent(int agtId, int objId)
	{
		int i, j, th, d;
		pose p1,p2;	// agt_cg, obj_cg
		std::vector<int> trail;
		bool cSpaceTmp[GRID_A][GRID_Y][GRID_X];
		
		p1.pos.x = (int)round(agt[agtId].p.pos.x);
		p1.pos.y = (int)round(agt[agtId].p.pos.y);
		p1.ori = (int)floor(agt[agtId].p.ori);
		for (th = 0; th < GRID_A; th++) {
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++)
					cSpaceTmp[th][i][j] = agt[agtId].cSpace[th][i][j];
			}
		}		
		for (d = 0; d < obj[objId].dks.size(); d++) {
			nodeQ.clear();
			trail.clear();
			iv = INFVAL;
			p2.pos.x = (int)round(obj[objId].dks[d].pos.x);
			p2.pos.y = (int)round(obj[objId].dks[d].pos.y);
			p2.ori = (int)floor(obj[objId].dks[d].ori);

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

	std::vector<int> pathObject(int objId)
	{
		int i, j, th;
		std::vector<int> trail;
		nodeQ.clear();
		iv = INFVAL;
		pose p1, p2;	// cg, target
		bool cSpaceTmp[GRID_A][GRID_Y][GRID_X];
		
		p1.pos.x = (int)round(obj[objId].p.pos.x);
		p1.pos.y = (int)round(obj[objId].p.pos.y);
		p1.ori = (obj[objId].p.ori);//(int)round
		for (th = 0; th < GRID_A; th++) {
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++)
					cSpaceTmp[th][i][j] = obj[objId].cSpace[th][i][j];
			}
		}
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
		for (th = 0; th < GRID_A; th++) {
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++)
					obj[objId].cSpace[th][i][j] = cSpaceTmp[th][i][j];
			}
		}
		if (trail.size() == 0)
			trail.push_back(SKIP);
		return trail;
	}

	// ############################################## @$ ##############################################
	
	void NextMove(object* o, pose p1, pose p2, int travelled, std::vector<int> trail)
	{
		int x = p1.pos.x, y = p1.pos.y, th = p1.ori, tCost;		
		if (x > 0 && o->cSpace[th][y][x-1]) {
			tCost = travelled + CMOVE_W + distM(p2.pos.x,p2.pos.y,x-1,y)
					+ (std::abs(p2.ori-th) > GRID_A/2 ? GRID_A-std::abs(p2.ori-th) : std::abs(p2.ori-th));
			o->cSpace[th][y][x-1] = false;	// better explored marking mechanism
			if (tCost < iv)
				EnqueNode(x-1,y,th,travelled+CMOVE_W,tCost,trail,MOVE_W);
		}
		if (x < GRID_X-1 && o->cSpace[th][y][x+1]) {
			tCost = travelled + CMOVE_E + distM(p2.pos.x,p2.pos.y,x+1,y)
					+ (std::abs(p2.ori-th) > GRID_A/2 ? GRID_A-std::abs(p2.ori-th) : std::abs(p2.ori-th));
			o->cSpace[th][y][x+1] = false;	// better explored marking mechanism
			if (tCost < iv)
				EnqueNode(x+1,y,th,travelled+CMOVE_E,tCost,trail,MOVE_E);
		}
		if (y > 0 && o->cSpace[th][y-1][x]) {
			tCost = travelled + CMOVE_N + distM(p2.pos.x,p2.pos.y,x,y-1)
					+ (std::abs(p2.ori-th) > GRID_A/2 ? GRID_A-std::abs(p2.ori-th) : std::abs(p2.ori-th));
			o->cSpace[th][y-1][x] = false;	// better explored marking mechanism
			if (tCost < iv)
				EnqueNode(x,y-1,th,travelled+CMOVE_N,tCost,trail,MOVE_N);
		}
		if (y < GRID_Y-1 && o->cSpace[th][y+1][x]) {
			tCost = travelled + CMOVE_S + distM(p2.pos.x,p2.pos.y,x,y+1)
					+ (std::abs(p2.ori-th) > GRID_A/2 ? GRID_A-std::abs(p2.ori-th) : std::abs(p2.ori-th));
			o->cSpace[th][y+1][x] = false;	// better explored marking mechanism
			if (tCost < iv)
				EnqueNode(x,y+1,th,travelled+CMOVE_S,tCost,trail,MOVE_S);
		}
		if (o->cSpace[(int)std::fmod(th-1+GRID_A,GRID_A)][y][x]) {
			tCost = travelled + CTURN_C + distM(p2.pos.x,p2.pos.y,x,y)
					+ (std::abs(p2.ori-std::fmod(th-1+GRID_A,GRID_A)) > GRID_A/2 ? GRID_A-std::abs(p2.ori-std::fmod(th-1+GRID_A,GRID_A)) : std::abs(p2.ori-std::fmod(th-1+GRID_A,GRID_A)));
			o->cSpace[(int)std::fmod(th-1+GRID_A,GRID_A)][y][x] = false;	// better explored marking mechanism
			if (tCost < iv)
				EnqueNode(x,y,(int)std::fmod(th-1+GRID_A,GRID_A),travelled+CTURN_C,tCost,trail,TURN_C);
		}
		if (o->cSpace[(int)std::fmod(th+1,GRID_A)][y][x]) {
			tCost = travelled + CTURN_AC + distM(p2.pos.x,p2.pos.y,x,y)
					+ (std::abs(p2.ori-std::fmod(th+1,GRID_A)) > GRID_A/2 ? GRID_A-std::abs(p2.ori-std::fmod(th+1,GRID_A)) : std::abs(p2.ori-std::fmod(th+1,GRID_A)));
			o->cSpace[(int)std::fmod(th+1,GRID_A)][y][x] = false;	// better explored marking mechanism
			if (tCost < iv)
				EnqueNode(x,y,(int)std::fmod(th+1,GRID_A),travelled+CTURN_AC,tCost,trail,TURN_AC);
		}
	}

	// ############################################## @$ ##############################################
	
	void EnqueNode(int x, int y, int th, int travelled, int total, std::vector<int> trail, int move)
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

	int distM(int x1, int y1, int x2, int y2)	// Manhattan Distance
	{
		if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0)
			return INFVAL;
		if (hCostInf[y2][x2] && hCostInfAllow)
			return std::abs(x1-x2) + std::abs(y1-y2) + INFVAL/100;
		return std::abs(x1-x2) + std::abs(y1-y2);
	}

	// ############################################## @$ ##############################################

	int distC(point p1, point p2)	// Cartesian Distance
	{
		return std::sqrt(std::pow(p1.x-p2.x,2) + std::pow(p1.y-p2.y,2));
	}

	// ############################################## @$ ##############################################

	void updateEndEfctr(object* o)
	{
		int i, j, k;
		pose d;
		o->dks.clear();

		d.pos.x = (int)round(o->p.pos.x);
		d.pos.y = (int)round(o->p.pos.y);
		d.ori = (int)floor(o->p.ori);
		o->dks.push_back(d);
		return;

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

	void filterDocks(int objId, int rot)
	{
		// Dock N-Agents (N = obj.weight) --> NP-Complete Vertex Cover
		// Best Docks (independent of N) --> Convex Hull Chan's Algorithm
		// Special case 2 agents (0 enclosing area) --> Rotating Calipers
		int i,j, ii = 0, jj = 0;
		for (i = 0; i < obj[objId].dks.size(); i++) {
			for (j = 0; j < obj[objId].dks.size(); j++) {
				if (distC(obj[objId].dks[ii].pos,obj[objId].dks[jj].pos) < distC(obj[objId].dks[i].pos,obj[objId].dks[j].pos)) {
					ii = i;
					jj = j;
				}
			}
		}
		for (j = 0; j < obj[objId].dks.size(); j++) {
			if (j != ii && j != jj) {
				obj[objId].dks.erase(obj[objId].dks.begin()+j);
				j--;
				ii--;
				jj--;
			}
		}
	}

	// ############################################## @$ ##############################################
	
	pose estimateErr(object* o)
	{
		// Error = EstimatePose - ActualPose
		// Error in range [0,0.5]
		pose p;
		p.pos.x = round(o->p.pos.x) - o->p.pos.x;
		p.pos.y = round(o->p.pos.y) - o->p.pos.y;
		p.ori = floor(o->p.ori) - o->p.ori;
		return p;
	}

	// ############################################## @$ ##############################################
	
	float senseWhlEnc(float cmd)
	{
		// For wheel slipping, actual distance travelled is always less than commanded distance
		float errfrac = 0.01;
		return cmd - cmd*errfrac*(double(rand())/double(RAND_MAX));
	}

	// ############################################## @$ ##############################################
	
	float actuateT(float cmd)
	{
		std::cout << "Communicate to Motor Driver --> \t(non-differential) value : " << cmd;
		// Translate command to PWM DC Motor signal duration here...
		msg.angular.z = 0;
		msg.linear.x = 10;
		pb[i].publish(msg);
		usleep(cmd*10000);	// Wait for actuation to get over
		msg.angular.z = 0;
		msg.linear.x = 0;
		pb[i].publish(msg);		
		return cmd;
	}

	// ############################################## @$ ##############################################
	
	float actuateR(float cmd)
	{
		std::cout << "Communicate to Motor Driver --> \t(differential) value : " << cmd;
		// Translate command to PWM DC Motor signal duration here...
		return cmd;
	}	

	// ############################################## @$ ##############################################

	void frameWorld()
	{
		int i, j, n, World[GRID_Y][GRID_X];
		turtlesim::Spawn::Request req;
		turtlesim::Spawn::Response resp;
		ros::ServiceClient rst = nh.serviceClient<turtlesim::Spawn>("resetWorld");
		if (paintWall) {
			paintWall = false;
			rst.call(req, resp);
			ros::ServiceClient scW = nh.serviceClient<turtlesim::Spawn>("drawWall");
			req.theta = 0;
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++) {
					if (!EnvMap[i][j]) {
						req.y = i * GRID_SZ;
						req.x = j * GRID_SZ;
						scW.call(req, resp);
					}								
				}
			}
		}
		ros::ServiceClient reinit = nh.serviceClient<turtlesim::Spawn>("resetObjects");
		reinit.call(req, resp);
		ros::ServiceClient scO = nh.serviceClient<turtlesim::Spawn>("drawObjs");
		req.theta = M_PI/2;	// for Box
		for (n = 0; n < NUM_OBJ; n++) {
			for (i = 0; i < obj[n].pts.size(); i++) {
				req.y = (int)round(obj[n].pts[i].y) * GRID_SZ;
				req.x = (int)round(obj[n].pts[i].x) * GRID_SZ;
				scO.call(req, resp);
			}
		}
		ros::ServiceClient scA = nh.serviceClient<turtlesim::Spawn>("drawAgts");
		for (n = 0; n < NUM_AGT; n++) {
			req.theta = 2*M_PI*agt[n].p.ori/GRID_A;
			for (i = 0; i < agt[n].pts.size(); i++) {
				req.y = (int)round(agt[n].pts[i].y) * GRID_SZ;
				req.x = (int)round(agt[n].pts[i].x) * GRID_SZ;
				scA.call(req, resp);
			}
		}
		ros::ServiceClient fupd = nh.serviceClient<turtlesim::Spawn>("frmUpdt");
		fupd.call(req,resp);
		//World[(int)round(obj[n].target.pos.y)][(int)round(obj[n].target.pos.x)] = TARGET;
		//for (i = 0; i < obj[n].dks.size(); i++) {
		//		World[(int)round(obj[n].dks[i].pos.y)][(int)round(obj[n].dks[i].pos.x)] = DOCK;
		//	}
	}

	// ############################################## @$ ##############################################

	void printObject(int n, bool agent)
	{
		if (agent) {
			std::cout <<  "Info. --> Agent " << n << std::endl;
			std::cout <<  " * Pose (x,y,th) : " << agt[n].p.pos.x << "," << agt[n].p.pos.y << "," << agt[n].p.ori << std::endl;
			std::cout <<  " * Points(docks) : " << agt[n].pts.size() <<  "(" << agt[n].dks.size() << ")" << std::endl;
			//std::cout <<  " * Weight : " << agt[n].wgt << std::endl;
			//std::cout <<  " * Active : " << agt[n].pushActive << std::endl;
			std::cout <<  " * Assigned Object : " << agt[n].asgnmnt << std::endl;
		}
		else {
			std::cout <<  "Info. --> Object " << n << std::endl;
			std::cout <<  " * Pose (x,y,th) : " << obj[n].p.pos.x << "," << obj[n].p.pos.y << "," << obj[n].p.ori << std::endl;
			std::cout <<  " * Points(docks) : " << obj[n].pts.size() <<  "(" << obj[n].dks.size() << ")" << std::endl;
			//std::cout <<  " * Weight : " << obj[n].wgt << std::endl;
			//std::cout <<  " * Active : " << obj[n].pushActive << std::endl;
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
		std::cout << "Simulation Time " << (tick+1) << std::endl;
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
	
	void printDebug()
	{
		int n;
		printWorld();
		for (n = 0; n < NUM_OBJ; n++)	
			printObject(n,false);
		for (n = 0; n < NUM_AGT; n++)	
			printObject(n,true);
		return;
		frameWorld();
	}

	// ############################################## @$ ##############################################
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pasdedeux");
	PasDeDeux pdd(argc, argv);
	pdd.Adagio();
	ros::shutdown();
	return 1; 
}
