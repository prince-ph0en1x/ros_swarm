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
	
	// Define custom World Map here...
		
	// Lab Map Setup 1: single agent, single object, 6x7 grid with 45 deg angular resolution.
		
		// x x x x x x x
		// x T T . . . x
		// x T T O O . x
		// x x . O O . x
		// x A . . . . x
		// x x x x x x x
		
	static const int GRID_Y = 6;				// Grid Sized Cells in Y direction
	static const int GRID_X = 7;				// Grid Sized Cells in X direction
	static const int GRID_A = 8;				// Angular Resolution (2*PI/GRID_A)	--> 4 >> 90deg, 8 >> 45deg, 16 >> 22.5deg, 36 >> 10deg  
	
	static const int NUM_OBJ = 1;				// Number of total target objects on map
	static const int NUM_AGT = 1;				// Number of total agents on same ROS master
	
	static const int GRID_SZ = 32;					// Qt Frame Grid Size in Pixels
	static const int WORLD_X = GRID_SZ * GRID_X;	// Column Pixels
	static const int WORLD_Y = GRID_SZ * GRID_Y;	// Row Pixels
};

#endif
