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
 
enum EnvStates {
	EMPTY = 0, OBSTACLE = 1, AGENT = 2, OBJECT = 3, PATH = 4, EXPLORED = 5, TARGET = 6
};

class WolframAutomata
{

public:

	static const int GRID_X = 140;
	static const int GRID_Y = 150;

	int World[GRID_X];
	
	
	void generate(int rule)
	{
		int i,j;
		for (i = 0; i < GRID_X; i++) {
			World[i] = EMPTY;
		}

		World[70] = OBSTACLE;

		for (i = 0; i < GRID_Y; i++) {
			evolveWorld();
			printWorld();
		}
		
	}

	void evolveWorld()
	{
		int i,temp[GRID_X];
		for (i = 0; i < GRID_X-2; i++) {
			temp[i+1] = OBSTACLE;
			if (World[i] == EMPTY && World[i+1] == EMPTY && World[i+2] == EMPTY)
				temp[i+1] = EMPTY;
			if (World[i] == OBSTACLE && World[i+1] == OBSTACLE && World[i+2] == OBSTACLE)
				temp[i+1] = EMPTY;
		}
		temp[0] = OBSTACLE;
		temp[i+1] = OBSTACLE;
		if (World[0] == EMPTY && World[1] == EMPTY)
			temp[0] = EMPTY;
		if (World[i] == EMPTY && World[i+1] == EMPTY)
			temp[i+1] = EMPTY;

		for (i = 0; i < GRID_X; i++) {
			World[i] = temp[i];
		}

	}

	void printWorld()
	{
		int i,j;
		//for (i = 0; i < GRID_Y; i++) {
			for (j = 0; j < GRID_X; j++) {
				if (World[j] == EMPTY) {
					std::cout << " ";
				}
				else if (World[j] == OBSTACLE) {
					std::cout << "#";
				}
				else if (World[j] == EXPLORED) {
					std::cout << ", ";
				}
				else if (World[j] == OBJECT) {
					std::cout << "[]";
				}
				else if (World[j] == AGENT) {
					std::cout << "# ";
				}
				else if (World[j] == TARGET) {
					std::cout << "x ";
				}
				else if (World[j] == PATH) {
					std::cout << "x ";
				}
				else {
					std::cout << World[j];
				}
			}
			std::cout << std::endl;
		//}
	}
};

int main(int argc, char** argv)
{
  WolframAutomata wa;
  wa.generate(126);
  return 1; 
}
