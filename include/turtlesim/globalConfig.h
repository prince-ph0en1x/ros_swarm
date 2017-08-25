/**
 * \file globalconfig.h
 * \legacy 'globalconfig.h'
 * 
 * \author Aritra Sarkar
 * \date 17-08-2017 (begin v2)
 */

#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

class Globals
{

public:
	
	static const int GRID_X = 20;				// Grid Sized Cells in X direction
	static const int GRID_Y = 20;				// Grid Sized Cells in Y direction
	static const int GRID_A = 8;				// Angular Resolution (2*PI/GRID_A)	--> 8-45d 16-22.5d 36-10d  
	
	static const int GRID_SZ = 32;				// Qt Frame Grid Size in Pixels
	
	static const int NUM_OBJ = 1;
	static const int NUM_AGT = 3;
	
	static const int WORLD_X = GRID_SZ * GRID_X;	// Column Pixels
	static const int WORLD_Y = GRID_SZ * GRID_Y;	// Row Pixels
};
#endif