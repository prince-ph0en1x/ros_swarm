/**
 * \file mezca.cpp
 * \legacy gol.cpp
 * 
 * \author Aritra Sarkar
 * \date 22-08-2017 (begin)
 */

// Maze solver using 6 state cellular automata

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <unistd.h>		// for usleep
#include <iomanip>
 
const int GRID_X = 15;//37;
const int GRID_Y = 15;//37;

const int SIM_TIME = 100;
const int SIM_DELAY = 200000;

enum Env {
	Black		= 0, 
	White		= 1, 
	Yellow		= 2, 
	Orange		= 3, 
	Red			= 4, 
	Grey		= 5	
};

class Automata
{

public:

	int EnvMap[GRID_Y][GRID_X];
	int Path[GRID_Y][GRID_X];
	int tick;
	int endx, endy;
	bool found;
	
	// ############################################## @$ ##############################################

	Automata()
	{
		tick = 0;
		int i,j;
		srand(time(0));
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				EnvMap[i][j] = White;
				Path[i][j] = 0;
			}
		}
		found = false;
		initWorld();
	}
	
	// ############################################## @$ ##############################################

	void life()
	{
		//for (; tick < SIM_TIME; tick++) {
		while(EnvMap[endy][endx] != Orange && tick < SIM_TIME) {
			tick++;
			printWorld();
			evolve();
		}
		printWorld();
		int i, j;
		for (i = 0; i < GRID_Y; i++) {
			std::cout << " ";
			for (j = 0; j < GRID_X; j++) {
				if (Path[i][j] >= Path[endy][endx])
					std::cout << " x";
				else
					std::cout << " .";
				//std::cout << std::setw(3) << std::setfill('0') << Path[i][j] << " ";
			}
			std::cout << std::endl;
		}
	}

	// ############################################## @$ ##############################################

private:
	
	void initWorld()
	{
		int i, j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++)
				if (double(rand())/double(RAND_MAX) > 0.8)  EnvMap[i][j] = Black;				
		}
		for (i = 0; i < GRID_Y; i++) {
			EnvMap[i][0] = Black;	
			EnvMap[i][GRID_X-1] = Black;			
		}
		for (i = 0; i < GRID_X; i++) {
			EnvMap[0][i] = Black;	
			EnvMap[GRID_Y-1][i] = Black;			
		}
		do {
			i = (int)round(GRID_Y * double(rand()) / double(RAND_MAX));
			j = (int)round(GRID_X * double(rand()) / double(RAND_MAX));
		} while(!EnvMap[i][j] == White);
		EnvMap[i][j] = Red;
		Path[i][j]++;
		do {
			endy = (int)round(GRID_Y * double(rand()) / double(RAND_MAX));
			endx = (int)round(GRID_X * double(rand()) / double(RAND_MAX));
		} while(!EnvMap[endy][endx] == White);
		EnvMap[endy][endx] = Yellow;
		Path[endy][endx]++;
	}

	// ############################################## @$ ##############################################
	
	void evolve()
	{
		int i, j, k;
		int World[GRID_Y][GRID_X];
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				// CA Rules
				switch (EnvMap[i][j]) {
					case Black		:	
						World[i][j] = Black;
						break;
					case White		:	
						World[i][j] = White;
						if (countNeighbours(i,j,Yellow) > 0 && 	found == false)	{
							World[i][j] = Yellow;
							Path[i][j]++;
						}
						if (countNeighbours(i,j,Black) > 2)						
							World[i][j] = Grey;
						break;
					case Yellow		:	
						World[i][j] = Yellow;
						if (countNeighbours(i,j,Red) > 0) {
							found = true;
							World[i][j] = Orange;
							Path[i][j]++;
						}
						if (countNeighbours(i,j,Orange) > 0)
							World[i][j] = Orange;
						if (found == false)
							Path[i][j]++;
						break;
					case Orange		:	
						Path[i][j]++;
						World[i][j] = Orange;
						break;
					case Red		:	
						World[i][j] = Red;
						if (found == true)
							Path[i][j]++;
						break;
					case Grey		:	
						World[i][j] = Grey;
						break;
					default			:	
						World[i][j] = EnvMap[i][j];
				}				
			}
		}
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++)
				EnvMap[i][j] = World[i][j];				
		}
	}

	// ############################################## @$ ##############################################

	int countNeighbours(int i, int j, int c)
	{
		int k = 0;
		// Von-Neumann Neighbours
		if (i > 1			&& EnvMap[i-1][j] == c)	k++;
		if (i < GRID_Y-1	&& EnvMap[i+1][j] == c)	k++;
		if (j > 1			&& EnvMap[i][j-1] == c)	k++;
		if (j < GRID_Y-1	&& EnvMap[i][j+1] == c)	k++;
		// Additional Moore Neighbours
		//if (i > 1			&& j > 1			&& EnvMap[i-1][j-1] == c)	k++;
		//if (i < GRID_Y-1	&& j > 1			&& EnvMap[i+1][j-1] == c)	k++;
		//if (i > 1			&& j < GRID_Y-1		&& EnvMap[i-1][j+1] == c)	k++;
		//if (i < GRID_Y-1	&& j < GRID_Y-1		&& EnvMap[i+1][j+1] == c)	k++;
		return k;
	}

	// ############################################## @$ ##############################################

	void printWorld()
	{
		int i, j;
		int b = 0, w = 0, y = 0, o = 0, r = 0, g = 0;
		usleep(SIM_DELAY);
		std::cout << "\033[2J\033[1;1H";
		std::cout << "Simulation Time " << (tick+1) << std::endl;
		std::cout << std::endl;
		for (i = 0; i < GRID_Y; i++) {
			std::cout << " ";
			for (j = 0; j < GRID_X; j++) {
				switch (EnvMap[i][j]) {
					case Black		:	std::cout << " B"; b++;	break;
					case White		:	std::cout << " ."; w++;	break;
					case Yellow		:	
						if (i == endy && j == endx)	std::cout << " X";
						else 						std::cout << " Y";
						y++;
						break;
					case Orange		:
						if (i == endy && j == endx)	std::cout << " X";
						else 						std::cout << " O";
						o++;
						break;
					case Red		:	std::cout << " R"; r++;	break;
					case Grey		:	std::cout << " G"; g++;	break;
					default			:	std::cout << EnvMap[i][j];
				}
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
		std::cout << "Cell Stats --> "<<std::endl;
		std::cout << "Black  : "<<b<<std::endl;
		std::cout << "White  : "<<w<<std::endl;
		std::cout << "Yellow : "<<y<<std::endl;
		std::cout << "Orange : "<<o<<std::endl;
		std::cout << "Red    : "<<r<<std::endl;
		std::cout << "Grey   : "<<g<<std::endl;
		std::cout << "Fraction   --> "<<(((float)100*o/(y+o+w+g)))<<"%" <<std::endl;

	}

	// ############################################## @$ ##############################################
};

int main(int argc, char** argv)
{
	Automata ca;
	ca.life();
	return 1; 
}