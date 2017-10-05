/**
 * \file PasDeDeuxSense.cpp
 * \legacy PasDeDeuxMcellBT.cpp
 * 
 * Internship Project at TCS Innovation Labs Kolkata
 * 
 * \author Aritra Sarkar
 * \date 07-08-2017 (begin)
 */

/* Core Algorithm :
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

const int SIM_TIME = 150;
const int SIM_DELAY = 100000;

class Automata
{

public:

	bool EnvMap[GRID_Y][GRID_X];
	int tick;
	
	// ############################################## @$ ##############################################

	Automata()
	{
		tick = 0;
		int i,j;
		srand(time(0));
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				EnvMap[i][j] = false;
			}
		}
		initWorld();
	}
	
	// ############################################## @$ ##############################################

	void life()
	{
		for (; tick < SIM_TIME; tick++) {
			printWorld();
			evolve();
		}
	}

	// ############################################## @$ ##############################################

private:
	
	void initWorld()
	{
		int i, j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++)
				if (double(rand())/double(RAND_MAX) > 0.5)  EnvMap[i][j] = true;				
		}
	}

	// ############################################## @$ ##############################################
	
	void evolve()
	{
		int i, j, k;
		bool World[GRID_Y][GRID_X];
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				k = 0;
				if (i > 1			&& EnvMap[i-1][j])	k++;
				if (i < GRID_Y-1	&& EnvMap[i+1][j])	k++;
				if (j > 1			&& EnvMap[i][j-1])	k++;
				if (j < GRID_Y-1	&& EnvMap[i][j+1])	k++;
				if (i > 1			&& j > 1			&& EnvMap[i-1][j-1])	k++;
				if (i < GRID_Y-1	&& j > 1			&& EnvMap[i+1][j-1])	k++;
				if (i > 1			&& j < GRID_Y-1		&& EnvMap[i-1][j+1])	k++;
				if (i < GRID_Y-1	&& j < GRID_Y-1		&& EnvMap[i+1][j+1])	k++;

				if (k < 2)		World[i][j] = false;
				else if (k < 3)	World[i][j] = EnvMap[i][j];
				else if (k < 4)	World[i][j] = true;
				else			World[i][j] = false;
								
			}
		}
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++)
				EnvMap[i][j] = World[i][j];				
		}
	}

	// ############################################## @$ ##############################################

	void printWorld()
	{
		int i, j;
		usleep(SIM_DELAY);
		std::cout << "\033[2J\033[1;1H";
		std::cout << "Simulation Time " << (tick+1) << std::endl;
		std::cout << std::endl;
		for (i = 0; i < GRID_Y; i++) {
			std::cout << " ";
			for (j = 0; j < GRID_X; j++) {
				if (EnvMap[i][j]) 	std::cout << "NN";
				else				std::cout << " .";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}

	// ############################################## @$ ##############################################
};

int main(int argc, char** argv)
{
	Automata ca;
	ca.life();
	return 1; 
}