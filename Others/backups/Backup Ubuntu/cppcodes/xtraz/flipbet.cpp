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

using namespace std;

class flipbet
{

public:

	static const int GRID_X = 140;
	static const int GRID_Y = 150;

	int World[GRID_X];
	
	
	void toss(int rule)
	{
		srand(time(0));
		int i,j,h=0,t=0,w=0,p;
		int tries = 10;
		for (i = 0; i < tries; i++) {
			j = (int)(rand()/(double)RAND_MAX*2);
			if (h > t) p = 0;	// Markov Prediction if Total Probability given
			else p = 1;
			if (j > 0) h++;
			else t++;

			if (p == j) w++;
			else w--;
		}
		cout << "H : " << h << " T : " << t << " W : " << w;
		
	}


};

int main(int argc, char** argv)
{
  flipbet fb;
  fb.toss(126);
  return 1; 
}
