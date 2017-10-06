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
		
	// Lab Map Setup 1: single agent, single object, 14x9 grid with 45 deg angular resolution.
		
	/*
	- 0 1 2 3 4 5 6 7 8
	0 x x x x x x x x x
	1 x x A . . . . . x
	2 x x x . . . . . x
	3 x . x . . x x x x 
	4 x . x . . x . . x 
	5 x . x . . x A . x 
	6 x x x . . x x . x
	7 x . . . . . x . x
	8 x . . O O . x . x
	9 x . . O O . x . X
	0 x . . . . . x . x
	1 x x x T T x . . x
	2 x . x T T x . . x
	3 x x x x x x x x x
	*/		
	
	static const int GRID_Y = 14;				// Grid Sized Cells in Y direction
	static const int GRID_X = 9;				// Grid Sized Cells in X direction
	static const int GRID_A = 4;				// Angular Resolution (2*PI/GRID_A)	--> 4 >> 90deg, 8 >> 45deg, 16 >> 22.5deg, 36 >> 10deg  
	static const int SCALE = 30;				// 1 Cell = 30cm x 30cm
	
	static const int NUM_OBJ = 1;				// Number of total target objects on map
	static const int NUM_AGT = 2;				// Number of total agents on same ROS master
	
	// Only required for Qt simulation
	static const int GRID_SZ = 32;					// Qt Frame Grid Size in Pixels
	static const int WORLD_X = GRID_SZ * GRID_X;	// Column Pixels
	static const int WORLD_Y = GRID_SZ * GRID_Y;	// Row Pixels
};

#endif
