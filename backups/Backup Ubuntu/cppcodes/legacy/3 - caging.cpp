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

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <cmath>
 
enum EnvStates {
	EMPTY = 0, OBSTACLE = 1, AGENT = 2, OBJECT = 3, PATH = 4, EXPLORED = 5, TARGET = 6, DOCK = 7, OBJCG = 8, POC = 9 
};

int infVal = 100000;

class CagingTestbed
{

public:

	static const int GRID_X = 40;
	static const int GRID_Y = 40;

	int EnvMap[GRID_Y][GRID_X];	// 0 - free cell, 1 - wall, 2 - agent, 3xx - object
	
	
	void cage()
	{
		int i,j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				EnvMap[i][j] = EMPTY;
			}
		}

		EnvMap[7][7] = TARGET;

		// Object 1
		for (i = 15; i < 15+3; i++) {
			for (j = 8; j < 10+7; j++) {
				EnvMap[i][j] = OBJECT;
			}
		}
		EnvMap[16][12] = OBJCG;

		
		// Object 2
		for (i = 10; i < 10+5; i++) {
			EnvMap[i][30] = OBJECT;
			EnvMap[i][34] = OBJECT;
		}
		for (j = 30; j < 30+5; j++) {
				EnvMap[14][j] = OBJECT;
		}
		EnvMap[12][32] = OBJCG;

		// Object 3
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (distC(i,j,30,30) < 5) {
					EnvMap[i][j] = OBJECT;
				}
			}
		}
		EnvMap[30][30] = OBJCG;

		EnvMap[30][8] = AGENT;
		EnvMap[30][15] = AGENT;

		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (EnvMap[i][j] == OBJECT) {
					if (i-1 > 0 && EnvMap[i-1][j] == EMPTY)
						EnvMap[i-1][j] = DOCK;
					if (i+1 < GRID_Y && EnvMap[i+1][j] == EMPTY)
						EnvMap[i+1][j] = DOCK;
					if (j-1 > 0 && EnvMap[i][j-1] == EMPTY)
						EnvMap[i][j-1] = DOCK;
					if (j+1 < GRID_X && EnvMap[i][j+1] == EMPTY)
						EnvMap[i][j+1] = DOCK;
				}
			}
		}

		printEnvMap();

		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (EnvMap[i][j] == DOCK) {
					if (i-1 > 0 && EnvMap[i-1][j] == OBJECT) {
						if (distM(12-1,12,3,3) < distM(12,12,3,3))
							EnvMap[i][j] = POC;
					}
					if (i+1 < GRID_Y && EnvMap[i+1][j] == OBJECT) {
						if (distM(12+1,12,3,3) < distM(12,12,3,3))
							EnvMap[i][j] = POC;
					}
					if (j-1 > 0 && EnvMap[i][j-1] == OBJECT) {
						if (distM(12,12-1,3,3) < distM(12,12,3,3))
							EnvMap[i][j] = POC;
					}
					if (j+1 < GRID_X && EnvMap[i][j+1] == OBJECT) {
						if (distM(12,12+1,3,3) < distM(12,12,3,3))
							EnvMap[i][j] = POC;
					}
				}
			}
		}



		std::cout << "\033[2J\033[1;1H";
		printEnvMap();
	}

	int distM(int x1, int y1, int x2, int y2)	// Manhattan Distance
	{
		if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0)
			return infVal;
		return std::abs(x1-x2) + std::abs(y1-y2);
	}

	int distC(int x1, int y1, int x2, int y2)	// Cartesian Distance
	{
		return std::sqrt(std::pow(x1-x2,2) + std::pow(y1-y2,2));
	}

	void printEnvMap()
	{
		int i,j;
		for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (EnvMap[i][j] == EMPTY) {
					std::cout << ". ";
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
				else if (EnvMap[i][j] == PATH) {
					std::cout << "x ";
				}
				else {
					std::cout << EnvMap[i][j];
				}
			}
			std::cout << std::endl;
		}
	}
};

int main(int argc, char** argv)
{
  CagingTestbed ctb;
  ctb.cage();
  return 1; 
}
