/**
 * \file pasdedeux.cpp
 * \legacy 'pasdedeux_sim.cpp'
 * 
 * \author Aritra Sarkar
 * \date 03-10-2017 (begin)
 */

/*
 * Execution Steps:
 * 
 * cd ~/Desktop/Aritra/swarm/
 * catkin_make
 * roscore
 * rosrun pas_de_deux swarm_node 0
 * rosrun pas_de_deux swarm_node 1
 * 
 */

#include "pasdedeux.h"


#define HI 1
#define LO 0

enum EnvStates {	// for printWorld grid map display
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
	
	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################
	

	
class control
{
	
public:

	static int mL1, mL2, mR2, mR1;	// motors
	
	static int eL, eR;				// encoders
	static int eL_val, eR_val;
	static int eL_lim, eR_lim;

	int diaWhl, diaAgt;
	int eSpok;

	// ############################################## @$ ##############################################
	
	void init()
	{
					//	J8		WPi		BCM		WireColour
					
		mL1 = 21;	//	29		21		5		black
		mL2 = 22;	//	31		22		6		white
		mR2 = 25;	//	35		25		26		red
		mR1 = 24;	//	37		24		19		brown

		eL = 3;		//	15		3		22		grey	(black-gnd white-vcc)
		eR = 8;		//	3		8		2		yellow	(black-gnd white-vcc)

		diaWhl = 5;		// cm
		diaAgt = 16;	// cm
		eSpok = 30;		// no. of triggers of each state (0,1) is eSpok
		
		eL_lim = 0;
		eR_lim = 0;
		eL_val = 0;
		eR_val = 0;
		
		wiringPiSetup();	

		pinMode(mL1,OUTPUT);	
		pinMode(mL2,OUTPUT);
		pinMode(mR1,OUTPUT);
		pinMode(mR2,OUTPUT);

		pinMode(eL,INPUT);	
		pinMode(eR,INPUT);

		pullUpDnControl(eL,PUD_UP);	// pull up is needed as encoder common is grounded	
		pullUpDnControl(eR,PUD_UP);		

		wiringPiISR(eL,INT_EDGE_BOTH,_senseLeftCallback);
		wiringPiISR(eR,INT_EDGE_BOTH,_senseRightCallback);	
	}

	// ############################################## @$ ##############################################
	
	static void _senseLeftCallback()
	{
		if (++eL_val >= eL_lim) {
			digitalWrite(mL1,LOW);
			digitalWrite(mL2,LOW);
		}
	}

	// ############################################## @$ ##############################################
	
	static void _senseRightCallback()
	{
		if (++eR_val >= eR_lim) {
			digitalWrite(mR1,LOW);
			digitalWrite(mR2,LOW);
		}
	}

	// ############################################## @$ ##############################################
	
	void left(float d)
	{
		eL_lim = 2*eSpok*((float)diaAgt*d/(diaWhl*2*M_PI)); 
		eR_lim = 2*eSpok*((float)diaAgt*d/(diaWhl*2*M_PI)); 
		std::cout << "L -- (" << eL_lim << "," << eR_lim << ") -- ";
		eL_val = 0;
		eR_val = 0;
		digitalWrite(mL2,LOW);
		digitalWrite(mR2,LOW);
		digitalWrite(mL1,HIGH);
		digitalWrite(mR1,HIGH);
		while (eL_val < eL_lim || eR_val < eR_lim) {
			if (eL_val < eL_lim)	digitalWrite(mL1,HIGH);
			if (eR_val < eR_lim)	digitalWrite(mR1,HIGH);
		}
		std::cout << "Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tD : " << d << std::endl;
	}

	// ############################################## @$ ##############################################
	
	void right(float d)
	{
		eL_lim = 2*eSpok*((float)diaAgt*d/(diaWhl*2*M_PI)); 
		eR_lim = 2*eSpok*((float)diaAgt*d/(diaWhl*2*M_PI)); 
		std::cout << "R -- (" << eL_lim << "," << eR_lim << ") -- ";
		eL_val = 0;
		eR_val = 0;
		digitalWrite(mL1,LOW);
		digitalWrite(mR1,LOW);
		digitalWrite(mL2,HIGH);
		digitalWrite(mR2,HIGH);
		while (eL_val < eL_lim || eR_val < eR_lim) {
			if (eL_val < eL_lim)	digitalWrite(mL2,HIGH);
			if (eR_val < eR_lim)	digitalWrite(mR2,HIGH);
		}
		std::cout << "Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tD : " << d << std::endl;
	}

	// ############################################## @$ ##############################################
	
	void back(float d)
	{
		eL_lim = 2*eSpok*((float)d/(diaWhl*M_PI));
		eR_lim = 2*eSpok*((float)d/(diaWhl*M_PI));
		std::cout << "B -- (" << eL_lim << "," << eR_lim << ") -- ";
		eL_val = 0;
		eR_val = 0;
		digitalWrite(mL1,LOW);
		digitalWrite(mR2,LOW);
		digitalWrite(mL2,HIGH);
		digitalWrite(mR1,HIGH);
		while (eL_val < eL_lim || eR_val < eR_lim) {
			if (eL_val < eL_lim)	digitalWrite(mL2,HIGH);
			if (eR_val < eR_lim)	digitalWrite(mR1,HIGH);
		}
		std::cout << "Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tD : " << d << std::endl;
	}

	// ############################################## @$ ##############################################
	
	void front(float d)
	{
		eL_lim = 2*eSpok*((float)d/(diaWhl*M_PI));
		eR_lim = 2*eSpok*((float)d/(diaWhl*M_PI));
		std::cout << "F -- (" << eL_lim << "," << eR_lim << ") -- ";
		eL_val = 0;
		eR_val = 0;
		digitalWrite(mL2,LOW);
		digitalWrite(mR1,LOW);
		digitalWrite(mL1,HIGH);
		digitalWrite(mR2,HIGH);
		while (eL_val < eL_lim || eR_val < eR_lim) {
			if (eL_val < eL_lim)	digitalWrite(mL1,HIGH);
			if (eR_val < eR_lim)	digitalWrite(mR2,HIGH);
		}
		std::cout << "Encoder Motion Estimate = L : " << eL_val << "\tR : " << eR_val << "\tD : " << d << std::endl;
	}
	
	// ############################################## @$ ##############################################
	
	void stop()
	{
		digitalWrite(mL2,LOW);
		digitalWrite(mL1,LOW);
		digitalWrite(mR1,LOW);
		digitalWrite(mR2,LOW);
	}
};
	
	int control::mL1, control::mL2, control::mR1, control::mR2;
	int control::eL, control::eR;
	int control::eL_val, control::eR_val, control::eL_lim, control::eR_lim;
	
	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################
	
bool INTELLIGENT;	// true if agent has capability of leader processing
// bool MOBILE;		// true if agent has capability of slave actuation
int ROS_NODE_ID;	// manual agent id assignment: 1-Alpha 2-Bravo 3-Charlie 4-Delta 5-Echo

// Core class handling both distributed leader-election phase and centralized collaborative-pushing phase
class PasDeDeux
{

public:
	void pollBooth(const geometry_msgs::Twist& voteMsg);	// save votes
	void actuate(const geometry_msgs::Twist& moveMsg);
	bool sense(pas_de_deux::Sensor::Request& req, pas_de_deux::Sensor::Response& res);
	bool leader;	// is the program instance simulating the leader
	bool mobile;	// is the leader mobile or standalone server
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
	ros::Subscriber sb;
	ros::ServiceClient cl;
	ros::ServiceServer sv[NUM_AGT];
	ros::NodeHandle nh;	
	bool paintWall;
	float polls[NUM_AGT];	// Poll values	
	bool syncSts[NUM_AGT];	// Global poll sync status
	bool rcvdPolls[NUM_AGT];// Local poll sync status
	float whlEnc[NUM_AGT][3];	// Left(0), Right(1) and Dirty Bit(2) for Wheel Encoder Reading
	control c;
			
	// ############################################## @$ ##############################################

	PasDeDeux(int argc, char** argv)
	{
		srand(time(0));
		// TESTMODE : leader = electLeader();
		if (ROS_NODE_ID == 0)
			leader = true;
		else
			leader = false;		
		msg.angular.z = 0;
		msg.linear.x = 0;
		c.init();
	}	
	
	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################
	
	bool electLeader()
	{
		/* array = [index = agtId][random polled value][received?][synced?]
		 * if capable == 1, poll own value, else 0
		 * own sync = 0
		 * while (sum of synced == NUM_AGT) {
		 * 		publish own value & sync
		 * 		check subscriptions and fill received == false
		 * 		if all received = true, own sync = 1
		 * }
		 * if own value is max value, leader, else slave
		 */
		int i; 
		bool synced = false;
		std::ostringstream oss;
		oss.str("");
		oss << "agt" << ROS_NODE_ID << "/msg_poll";
		ros::Publisher pbVote = nh.advertise<geometry_msgs::Twist>(oss.str(),100);
		usleep(50000);	// wait till publisher is registered with master
		ros::Subscriber sbVote[NUM_AGT];
		for(i = 0; i < NUM_AGT; i++) {
			if (ROS_NODE_ID == i)
				continue;
			oss.str("");
			oss << "agt" << i << "/msg_poll";
			sbVote[i] = nh.subscribe(oss.str(),100,&PasDeDeux::pollBooth,this);
			usleep(10000);
		}					
		for (i = 0; i < NUM_AGT; i++) {
			rcvdPolls[i] = false;
			syncSts[i] = false;
		}
		polls[ROS_NODE_ID] = INTELLIGENT ? double(rand()) / double(RAND_MAX) : 0;
		rcvdPolls[ROS_NODE_ID] = true;
		std::cout << "Entering Poll Loop " << std::endl;
		do {
			synced = true;
			for (i = 0; i < NUM_AGT; i++) {
				if (syncSts[i] == false) {
					synced = false;
					break;
				}
			}
			msg.linear.x = ROS_NODE_ID;
			msg.linear.y = syncSts[ROS_NODE_ID] ? 1 : 0;
			msg.linear.z = polls[ROS_NODE_ID];
			pbVote.publish(msg);		
			ros::spinOnce();
			syncSts[ROS_NODE_ID] = true;
			for (i = 0; i < NUM_AGT; i++) {
				if (rcvdPolls[i] == false) {
					syncSts[ROS_NODE_ID] = false;
					break;
				}					
			}			
			for (i = 0; i < NUM_AGT; i++)
				std::cout << "Agent : " << i << "\tReceived : " << (rcvdPolls[i] ? 1 : 0) << "\tSynced : " << (syncSts[i] ? 1 : 0) << "\tPoll Value : " << polls[i] << std::endl;
			std::cout << std::endl;
			// sbVote[i].shutdown();
			usleep(2000);
		} while (!synced);
		std::cout << "Exiting Poll Loop " << std::endl;
		
		float leadVote = 0;
		int leadNode = INFVAL;
		for (i = 0; i < NUM_AGT; i++) {
			if (rcvdPolls[i] > leadVote) {
				leadVote = rcvdPolls[i];
				leadNode = i;
			}
		}
		for (int i = 0; i < NUM_AGT; i++)
			std::cout << "Agent : " << i << "\tPoll Value : " << polls[i] << std::endl;
		if (ROS_NODE_ID == leadNode) {
			std::cout << "Node " << ROS_NODE_ID << " is the master" << std::endl;
			//return true;
		}
		std::cout << "Node " << ROS_NODE_ID << " is a slave" << std::endl;
		//return false;		
		if (ROS_NODE_ID == 1) return true;
		else return false;	
	}	
	
	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################
	
	void Waltz()
	{	
		std::ostringstream oss;
		oss.str("");
		oss << "agt" << ROS_NODE_ID << "/msg_actuate";
		sb = nh.subscribe(oss.str(),1,&PasDeDeux::actuate,this);
		usleep(10000);
		oss.str("");
		oss << "agt" << ROS_NODE_ID << "/svc_sensor";
		cl = nh.serviceClient<pas_de_deux::Sensor>(oss.str());
		usleep(10000);
		std::cout << "ROS_NODE " << ROS_NODE_ID << " entering slave mode..." << std::endl;
		ros::spin();
		std::cout << "\nROS_NODE " << ROS_NODE_ID << " exiting slave mode..." << std::endl;
	}
		
	// ############################################## @$ ##############################################

	void Adagio()
	{
		int i, j, k, n, agtrr, manhandle;
		float cmdT, cmdR;
		pose objTgt, err;
		std::vector<int> agttrail[NUM_AGT];
		std::vector<int> objtrail[NUM_OBJ];
		std::vector<int> objtrailBkup, objtrailNew;		
		bool solve = false;		
		tick = 0;
		
		initWall();
		std::cout << "Number of Objects simulated = " << NUM_OBJ << std::endl;
		std::cout << "Number of Agents simulated = " << NUM_AGT << std::endl;		
		std::ostringstream oss;
		for(i = 0; i < NUM_AGT; i++) {
			oss.str("");
			oss << "agt" << i << "/msg_actuate";
			pb[i] = nh.advertise<geometry_msgs::Twist>(oss.str(),1);
			usleep(10000);
			pb[i].publish(msg);	// First msg gets missed -- WATSON
		}
		oss.str("");
		oss << "agt" << ROS_NODE_ID << "/msg_actuate";
		sb = nh.subscribe(oss.str(),1,&PasDeDeux::actuate,this);
		usleep(10000);
		
		for(i = 0; i < NUM_AGT; i++) {
			oss.str("");
			oss << "agt" << i << "/svc_sensor";
			sv[i] = nh.advertiseService(oss.str(),&PasDeDeux::sense,this);
			usleep(10000);
		}		
		oss.str("");
		oss << "agt" << ROS_NODE_ID << "/svc_sensor";
		cl = nh.serviceClient<pas_de_deux::Sensor>(oss.str());
		usleep(10000);

		std::cout << "Enter 1 for Stepwise Manual Correction Mode" << std::endl;
		std::cin >> manhandle;
		
		initObjects();	// initialize objects based on known map
		initAgents();	// initialize agents based on known map
		printDebug();
		assign();		// assign agent to objects
		for (n = 0; n < NUM_OBJ; n++)	
			buildCSpace(&obj[n]);	// build configuration space of each object
		hCostInfAllow = true;		// allow learning of (object+agent) non-navigable paths in map
		for (n = 0; n < NUM_OBJ; n++)				
			objtrail[n] = pathObject(n);	// find A* path of object
		hCostInfAllow = false;
		for (n = 0; n < NUM_OBJ; n++) {
			if (objtrail[n][0] == SKIP) {		// no A* path found for object
				std::cout << "No path found for object" << std::endl;
				continue;
			}
			objTgt = obj[n].target;			// temporarily store object target, as intermediate targets will be just the next grid step
			for (k = 0; k < objtrail[n].size();k++) {	// for each object movement step	
				updateDocks(&obj[n]);		// find pushable points of object (not necessarily in direction of required motion)
				obj[n].target = obj[n].p;	// set next grid step as target
				switch (objtrail[n][k]) {	// convert from motion step to coordinates
					case MOVE_E		:	obj[n].target.pos.x++;	break;
					case MOVE_N		: 	obj[n].target.pos.y--;	break;
					case MOVE_W		: 	obj[n].target.pos.x--;	break;
					case MOVE_S		:	obj[n].target.pos.y++;	break;
					case TURN_C		: 	obj[n].target.ori = std::fmod(obj[n].target.ori-1+GRID_A,GRID_A);	break;
					case TURN_AC	: 	obj[n].target.ori = std::fmod(obj[n].target.ori+1,GRID_A);	break;
				}
				selectDocks(&obj[n]);			// select pushing points in required direction of motion
				filterDocks(n,objtrail[n][k]);	// select best points equal to the number of agents
				printDebug();
				// IMPROVE IMP : Parallelize this loop - Multi-threading
				for (agtrr = 0; agtrr < NUM_AGT; agtrr++) {
					std::cout << "Agent " << agtrr << " assigned to " << agt[agtrr].asgnmnt << std::endl;
					if (agt[agtrr].asgnmnt != n)	// check if agent is assigned to object
						continue;
					buildCSpace(&agt[agtrr]);		// build configuration space of agent
					updateEndEfctr(&agt[agtrr]);	// IMPROVE : Merge functionality with updateDocks(&agt[agtrr]);
					agttrail[agtrr] = movesAgent(agtrr,pathAgent(agtrr,n));	// find A* path of agent to any filtered dock
					printTrail(agttrail[agtrr]);
					if (agttrail[agtrr].size() == 1 && agttrail[agtrr][0] == SKIP) {	// no A* path found for agent
						continue;
					}
					else if (agttrail[agtrr].size() != 0) {
						err = estimateErr(&agt[agtrr]);	// adjust for rounding error (real world vs grid world)
						cmdT = 0;
						cmdR = 0;
						// take current orientation into account
						switch (agttrail[agtrr][0]) {
							case MOVE_E		: cmdT = 1-err.pos.x;		break;
							case MOVE_N		: cmdT = 1-err.pos.y;		break;
							case MOVE_W		: cmdT = 1-err.pos.x;		break;
							case MOVE_S		: cmdT = 1-err.pos.y;		break;
							case TURN_C		: cmdR = -(1-err.ori);		break;
							case TURN_AC	: cmdR = 1-err.ori;			break;
						}
						if (manhandle == 1) {
							std::cout << "Continue pose correction?" << std::endl;		
							std::cin >> manhandle;
						}
						actuateCmd(agtrr,cmdT,cmdR);	// issue movement command to slave
						senseWhlEnc(agtrr,cmdT,cmdR);	// sense wheel encoder to calculate actual command execution status - IMPROVE IMP : make this wait in a seperate thread and proceed with next agent commanding
						std::cout << "World Map Update : T " << cmdT << "\tR : " << cmdR << std::endl;
						switch (agttrail[agtrr][0]) {	// update local map based on actual agent motion
							case MOVE_E		: translateObject(&agt[agtrr],cmdT,0);	break;
							case MOVE_N		: translateObject(&agt[agtrr],0,-cmdT);	break;
							case MOVE_W		: translateObject(&agt[agtrr],-cmdT,0);	break;
							case MOVE_S		: translateObject(&agt[agtrr],0,cmdT);	break;
							case TURN_C		: rotateObject(&agt[agtrr],cmdR);		break;
							case TURN_AC	: rotateObject(&agt[agtrr],cmdR);		break;
						}
						printDebug();
						std::cout << "\nObject " << n << " ";
						printTrail(objtrail[n]);	//printTrail(agttrail[agtrr]);
						agtrr--;
						continue;
					}
					agt[agtrr].pushActive = true;	// if agent reached dock, next step is pushing									
				}
				if (dockingComplete(n)) {	// if all assigned agents are docked
					switch (objtrail[n][k]) {	// move object to next grid
						case MOVE_E		: translateObject(&obj[n],1,0);		break;
						case MOVE_N		: translateObject(&obj[n],0,-1);	break;
						case MOVE_W		: translateObject(&obj[n],-1,0);	break;
						case MOVE_S		: translateObject(&obj[n],0,1);		break;
						case TURN_C		: rotateObject(&obj[n],-1);			break;
						case TURN_AC	: rotateObject(&obj[n],1);			break;
					}
					for (i = 0; i < NUM_AGT; i++) {
						if (agt[i].asgnmnt == n)
							agt[i].pushActive = false;	// make all assigned agents to non pushing location for next step
					}
					if (k == objtrail[n].size()-1)	// if object has reached the target
						solve = true;
				}
				else {
					hCostInf[(int)round(obj[n].p.pos.y)][(int)round(obj[n].p.pos.x)] = true;	// increase cost of current object location -- IMPROVE : different orientation different cost
					objtrailBkup = objtrail[n];
					obj[n].target = objTgt;	
					hCostInfAllow = true;
					// backtrace object motion to last path fork
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
			obj[n].dks.clear();		// clear object docking points for next iteration
		}
		printDebug();
		if (solve)
			std::cout << "Maze solvable in " << (tick+1) << " tries" <<"\n";
		else
			std::cout << "Maze not solvable in " << (tick+1) << " tries" <<"\n";
	}

	// ############################################## @$ ##############################################

private:
	
	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################

	void initWall()
	{
		int i, j, k;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				EnvMap[i][j] = true;		// make entire map navigable
				hCostInf[i][j] = false;		// make all free cells accessable with no extra cost for agent occupation while pushing
			}
		}
		// IMPROVE : Automate by read from a map file
		// define arena boundary
		for (k = 0; k < GRID_X; k++) {	// left right boundary wall
			EnvMap[0][k] = false;
			EnvMap[GRID_Y-1][k] = false;
		}
		for (k = 0; k < GRID_Y; k++) {	// top bottom boundary wall
			EnvMap[k][0] = false;
			EnvMap[k][GRID_X-1] = false;
		}
		// define obstacle islands
		EnvMap[1][1] = false;
		EnvMap[2][1] = false;
		EnvMap[6][1] = false;
		EnvMap[11][1] = false;
		EnvMap[2][2] = false;
		EnvMap[3][2] = false;
		EnvMap[4][2] = false;
		EnvMap[5][2] = false;
		EnvMap[6][2] = false;
		EnvMap[11][2] = false;
		EnvMap[12][2] = false;
		EnvMap[3][5] = false;
		EnvMap[4][5] = false;
		EnvMap[5][5] = false;
		EnvMap[6][5] = false;
		EnvMap[11][5] = false;
		EnvMap[12][5] = false;
		EnvMap[3][6] = false;
		EnvMap[6][6] = false;
		EnvMap[7][6] = false;
		EnvMap[8][6] = false;
		EnvMap[9][6] = false;
		EnvMap[10][6] = false;
		EnvMap[3][7] = false;
		
		paintWall = true;	// for Qt FrameWorld paint wall first time, then update only turtles/agents
	}

	// ############################################## @$ ##############################################
	
	void initObjects()
	{
		int i, j, o;
		int szx = 2, szy = 2;	// object length and breadth
		float x, y, th;
		point z;
		for (o = 0; o < NUM_OBJ; o++) {	// initialize each object
			// define object of specified size at origin
			obj[o].pts.clear();	// clear any grid points belonging to object
			obj[o].dks.clear();	// clear any dock points belonging to object
			for (i = 0; i < szy; i++) {
				for (j = 0; j < szx; j++) {
					z.y = i;
					z.x = j;
					obj[o].pts.push_back(z);
				}
			}
			// find object centroid
			obj[o].p.pos.x = 0;
			obj[o].p.pos.y = 0;
			for (i = 0; i < obj[o].pts.size(); i++) {
				obj[o].p.pos.x += obj[o].pts[i].x;
				obj[o].p.pos.y += obj[o].pts[i].y;
			}
			obj[o].p.pos.x = round(obj[o].p.pos.x/obj[o].pts.size());
			obj[o].p.pos.y = round(obj[o].p.pos.y/obj[o].pts.size());
			obj[o].p.ori = 0;	// default orientation (does not matter)
			buildCSpace(&obj[o]);	// build configuration space
			/*
			// find a random location where object can be placed without hinderance
			do {
				y = (int)round(GRID_Y * double(rand()) / double(RAND_MAX));
				x = (int)round(GRID_X * double(rand()) / double(RAND_MAX));
				th = 0;//(int)floor(GRID_A * double(rand()) / double(RAND_MAX));
			} while(!obj[o].cSpace[(int)round(th)][(int)round(y)][(int)round(x)]);
			*/
			// place object at known location
			y = 9;
			x = 4;
			th = 0;
			translateObject(&obj[o],x-obj[o].p.pos.x,y-obj[o].p.pos.y);
			rotateObject(&obj[o],th);
			obj[o].wgt = -1;			// weight is the number of agents required to push it. Object wt = -ve, Agent wt = +ve
			obj[o].pushActive = false;	// agents are not in place to push object
			// object's centroid target
			obj[o].target.pos.x = 4;
			obj[o].target.pos.y = 12;	
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
			/*
			do {
				z.y = (GRID_Y/10) * double(rand()) / double(RAND_MAX);
				z.x = (GRID_X/10) * double(rand()) / double(RAND_MAX);
			} while(!EnvMap[(int)round(z.y)][(int)round(z.x)]);
			*/
			if (a == 0) {
				z.y = 5;
				z.x = 6;
			}
			else if (a == 1) {
				z.y = 1;
				z.x = 2;
			}
			agt[a].pts.push_back(z);

			agt[a].p.ori = 0; //GRID_A * double(rand())/double(RAND_MAX);
			agt[a].p.pos.y = 0;
			agt[a].p.pos.x = 0;
			for (i = 0; i < agt[a].pts.size(); i++) {
				agt[a].p.pos.y += agt[a].pts[i].y;
				agt[a].p.pos.x += agt[a].pts[i].x;
			}
			agt[a].p.pos.y /= agt[a].pts.size();
			agt[a].p.pos.x /= agt[a].pts.size();
			
			agt[a].dks.clear();
			if (a != 0)
				agt[a].wgt = 1;
			else
				agt[a].wgt = 0;
			agt[a].pushActive = false;
			agt[a].asgnmnt = INFVAL;
		}
	}

	// ############################################## @$ ##############################################
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
			World[(int)round(o->pts[i].y)][(int)round(o->pts[i].x)] = EnvMap[(int)round(o->pts[i].y)][(int)round(o->pts[i].x)];
		for (i = 0; i < o->dks.size(); i++)
			World[(int)round(o->dks[i].pos.y)][(int)round(o->dks[i].pos.x)] = EnvMap[(int)round(o->dks[i].pos.y)][(int)round(o->dks[i].pos.x)];
		
		for (deg = 0; deg < GRID_A; deg++) {
			th = (int)floor(o->p.ori);
			for (i = 0; i < GRID_Y; i++) {
				for (j = 0; j < GRID_X; j++) {
					o->cSpace[th][i][j] = true;
					for (k = 0; k < o->pts.size(); k++) {
						dx = j - o->p.pos.x + round(o->pts[k].x);
						dy = i - o->p.pos.y + round(o->pts[k].y); 
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
	
	bool dockingComplete(int objId)
	{
		for (int a = 0; a < NUM_AGT; a++) {
			if (agt[a].asgnmnt == objId && agt[a].pushActive == false)
				return false;
		}
		return true;
	}

	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################
	
	std::vector<int> movesAgent(int agtId, std::vector<int> path)
	{
		int p, ori = agt[agtId].p.ori, rstep;
		std::vector<int>::iterator pi;
		
		for (p = 0; p < path.size();p++) {
			switch (path[p]) {
				case MOVE_E	:	// fall through for ActionObj enum path[p] = 0
				case MOVE_N	:	// fall through for ActionObj enum path[p] = 1
				case MOVE_W	:	// fall through for ActionObj enum path[p] = 2
				case MOVE_S	:	// fall through for ActionObj enum path[p] = 3
								rstep = (int)std::fmod(ori-path[p]*GRID_A/4+GRID_A,GRID_A);
								if (rstep == 0) {
									break;
								}
								ori = (int)std::fmod(ori-rstep+GRID_A,GRID_A);
								pi = path.begin()+p;
								if (rstep < GRID_A/2) {
									path.insert(pi,rstep,TURN_C);
									p += rstep;
								}
								else {
									path.insert(pi,GRID_A-rstep,TURN_AC);
									p += GRID_A-rstep;
								}								
								break;
				case TURN_C	:	ori = (int)std::fmod(ori-1+GRID_A,GRID_A);	break;
				case TURN_AC:	ori = (int)std::fmod(ori+1+GRID_A,GRID_A);	break;
				
			}
		}
		//if (path[path.size()-1]== TURN_C || path[path.size()-1] == TURN_AC)
		//		path.erase(path.end()-1);		// IMPROVE : Remove need for this in pathAgent
		return path;
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
		/*
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
		*/
		// Special case 1 agent (midpoint, object rotation not possible) --> Nearest to centre
		int i,j, ii = 0, jj = 0;
		jj = distC(obj[objId].dks[0].pos,obj[objId].p.pos);
		for (i = 0; i < obj[objId].dks.size(); i++) {
			if (distC(obj[objId].dks[i].pos,obj[objId].p.pos) < jj) {
				ii = i;
				jj = distC(obj[objId].dks[i].pos,obj[objId].p.pos);
			}
		}
		for (j = 0; j < obj[objId].dks.size(); j++) {
			if (j != ii) {
				obj[objId].dks.erase(obj[objId].dks.begin()+j);
				j--;
				ii--;
			}
		}
	}

	// ############################################## @$ ##############################################
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
					std::cout << " .";
				else
					std::cout << "HH";
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
		//std::cout << "Simulation Time " << (tick+1) << std::endl;
		std::cout << "\nWorld Map..." << std::endl;
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
		return;
		for (n = 0; n < NUM_OBJ; n++)	
			printObject(n,false);
		for (n = 0; n < NUM_AGT; n++)	
			printObject(n,true);
		frameWorld();	// suppressed in Raspberry Pi
	}
	
	// ############################################## @$ ##############################################
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

	void actuateCmd(int agtId, float cmdT, float cmdR)
	{
		std::cout << "Communicate to Motor Driver of Agent "<< agtId << " --> T : " << cmdT << "\tR : " << cmdR << std::endl;
		// commands are in number of grids in configuration space 3D matrix
		msg.linear.x = cmdT;
		msg.angular.z = cmdR;
		pb[agtId].publish(msg);
		usleep(10000);
	}
	
	// ############################################## @$ ##############################################
	
	void senseWhlEnc(int agtId, float& cmdT, float& cmdR)
	{
		// for wheel slipping, actual distance travelled can be less than commanded distance
		// for motor intertia, actual distance travelled can be more than commanded distance
		std::cout << "Processing Sensor Data of Agent " << agtId << std::endl;
		while (whlEnc[agtId][2] != 1)	// wait for dirty bit of agt to be set
			ros::spinOnce();			// IMPROVE : Insert waiting time threshold before Slave is dropped here...
		std::cout << "\033[2J\033[1;1H" << std::endl;	// clear screen
		if (cmdR == 0) {	// translation command
			cmdT = ((whlEnc[agtId][0]+whlEnc[agtId][1])/2)*(c.diaWhl*M_PI)/(2*c.eSpok*SCALE);
			cmdR = ((whlEnc[agtId][0]-whlEnc[agtId][1])/2)*(c.diaWhl*2*M_PI)/(2*c.eSpok*c.diaAgt*SCALE);
		}
		else { 			// rotation command
			cmdT = ((whlEnc[agtId][0]-whlEnc[agtId][1])/2)*(c.diaWhl*M_PI)/(2*c.eSpok*SCALE);
			if (cmdR > 0)
				cmdR = ((whlEnc[agtId][0]+whlEnc[agtId][1])/2)*c.diaWhl*GRID_A/(2*c.eSpok*c.diaAgt);
			else
				cmdR = -((whlEnc[agtId][0]+whlEnc[agtId][1])/2)*c.diaWhl*GRID_A/(2*c.eSpok*c.diaAgt);
		}
		whlEnc[agtId][2] = 0;	// reset dirty bit
	}
	
	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################
};

	void PasDeDeux::actuate(const geometry_msgs::Twist& moveMsg)
	{
		bool success;
		pas_de_deux::Sensor::Request req;
		pas_de_deux::Sensor::Response res;
		req.id = ROS_NODE_ID;
		if (moveMsg.linear.x == 0) {
			ROS_INFO_STREAM("Cmd Received : Rotate    = " << moveMsg.angular.z);
			if (moveMsg.angular.z > 0) 
				c.left(moveMsg.angular.z * 2 * M_PI / GRID_A);
			else
				c.right(abs(moveMsg.angular.z) * 2 * M_PI / GRID_A);
		}
		else {
			ROS_INFO_STREAM("Cmd Received : Translate = " << moveMsg.linear.x);
			ROS_INFO_STREAM("Scale = " << SCALE);
			c.front(moveMsg.linear.x * SCALE);
		}
		
		req.left = control::eL_val;
		req.right = control::eR_val;
		std::cout << "Sending Sensor readings to Leader : (L,R) = " << control::eL_val << "," << control::eR_val << std::endl;
		if (!leader)	success = cl.call(req,res);
		else			success = sense(req,res);
		std::cout << "Success? " << (success ? "true" : "false")  << "\nAwaiting next motion command... " << std::endl;
	}
	
	// ############################################## @$ ##############################################
	
	bool PasDeDeux::sense(pas_de_deux::Sensor::Request& req, pas_de_deux::Sensor::Response& res)
	{
		// WATSON : Handle case where dirty bit is already set
		std::cout << "Receiving Sensor readings from Agent " << req.id << std::endl;
		whlEnc[req.id][0] = req.left;
		whlEnc[req.id][1] = req.right;
		whlEnc[req.id][2] = 1;	// Set Motion Dirty
		return true;
	}
	
	// ############################################## @$ ##############################################
	
	void PasDeDeux::pollBooth(const geometry_msgs::Twist& voteMsg)
	{
		rcvdPolls[(int)voteMsg.linear.x] = true;	
		polls[(int)voteMsg.linear.x] = voteMsg.linear.z;
		syncSts[(int)voteMsg.linear.x] = voteMsg.linear.y == 1 ? true : false;	
	}

	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################

	int main(int argc, char** argv)
	{
		if (argc < 2) {
			std::cout << "Not enough input arguments --> rosrun pas_de_deux swarm_node <ROS_NODE_ID>" << std::endl;
			return 1;
		}
		ROS_NODE_ID = std::atoi(argv[1]);
		std::cout << "ROS_NODE_ID = " << ROS_NODE_ID << std::endl;
		std::ostringstream oss;
		oss.str("");
		oss << "pasdedeux" << ROS_NODE_ID;
		ros::init(argc, argv, oss.str());
		INTELLIGENT = true;
		PasDeDeux pdd(argc, argv);
		if (pdd.leader)
			pdd.Adagio();
		else
			pdd.Waltz();
		ros::shutdown();
		return 1; 
	}
	
	// ############################################## @$ ##############################################
	// ############################################## @$ ##############################################
