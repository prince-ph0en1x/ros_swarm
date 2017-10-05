#include <iostream>
#include <cmath>
#include <vector>


enum MARKER {
	MARK_EMPTY=0, MARK_OBSTACKLE=1, MARK_PATH=2, MARK_EXPLORED=3
};


enum MOVE {
	LEFT_MOVE=1, RIGHT_MOVE=2, UP_MOVE=3, DOWN_MOVE=4
};

#define LEFT_COST		1
#define RIGHT_COST		1
#define UP_COST			1
#define DOWN_COST		1


#define AREA_HEIGHT		52
#define AREA_WIDTH		50




typedef struct pp {
	int x, y;
} point;


int mat[AREA_HEIGHT][AREA_WIDTH];

typedef struct {
	point p;
	int traveled;
	int heuristic;
	std::vector<int> trail;
} node;



std::vector<node> nodeQ;

int goalDistance = 100000;

/*###############################################################*/
void EnqueuNode(int x, int y, int traveled, int heuristic, std::vector<int> trail, int move) {
	
	node n1, n2;
	trail.push_back(move);
	n1.p.x   = x;
	n1.p.y   = y;
	n1.trail = trail;
	n1.traveled  = traveled;
	n1.heuristic = heuristic;
	nodeQ.push_back(n1);

	for(int i=nodeQ.size()-2; i>=0; i--) {
		if(n1.heuristic < nodeQ[i].heuristic) {
			nodeQ[i+1].p.x   = nodeQ[i].p.x;
			nodeQ[i+1].p.y   = nodeQ[i].p.y;
			nodeQ[i+1].trail = nodeQ[i].trail;
			nodeQ[i+1].traveled  = nodeQ[i].traveled;
			nodeQ[i+1].heuristic = nodeQ[i].heuristic;
			if(i==0) {
				nodeQ[i].p.x   = n1.p.x;
				nodeQ[i].p.y   = n1.p.y;
				nodeQ[i].trail = n1.trail;
				nodeQ[i].traveled  = n1.traveled;
				nodeQ[i].heuristic = n1.heuristic;	
			}
		} else {
			nodeQ[i+1].p.x   = n1.p.x;
			nodeQ[i+1].p.y   = n1.p.y;
			nodeQ[i+1].trail = n1.trail;
			nodeQ[i+1].traveled  = n1.traveled;
			nodeQ[i+1].heuristic = n1.heuristic;
			break;
		}
	}
}

void NextMove(point p1, point p2, int traveled, std::vector<int> trail) {

	int x = p1.x;
	int y = p1.y;
	
	if(x>0 && mat[x-1][y]==0) {
		int heuristic = traveled + LEFT_COST + std::abs(p2.x-(x-1)) + std::abs(p2.y-y);
		if(heuristic < goalDistance) {
			EnqueuNode(x-1, y, traveled+LEFT_COST, heuristic, trail, LEFT_MOVE);
		}
		mat[x-1][y] = MARK_EXPLORED;
	}

	if(x<AREA_HEIGHT-1 && mat[x+1][y]==0) {
		int heuristic = traveled + RIGHT_COST + std::abs(p2.x-(x+1)) + std::abs(p2.y-y);
		if(heuristic < goalDistance) {
			EnqueuNode(x+1, y, traveled+RIGHT_COST, heuristic, trail, RIGHT_MOVE);
		}
		mat[x+1][y] = MARK_EXPLORED;
	}

	if(y>0 && mat[x][y-1]==0) {
		int heuristic = traveled + UP_COST + std::abs(p2.x-x) + std::abs(p2.y-(y-1));
		if(heuristic < goalDistance) {
			EnqueuNode(x, y-1, traveled+UP_COST, heuristic, trail, UP_MOVE);
		}
		mat[x][y-1] = MARK_EXPLORED;
	}
		
	if(y<AREA_WIDTH-1 && mat[x][y+1]==0) {
		int heuristic = traveled + DOWN_COST + std::abs(p2.x-x) + std::abs(p2.y-(y+1));
		if(heuristic < goalDistance) {
			EnqueuNode(x, y+1, traveled+DOWN_COST, heuristic, trail, DOWN_MOVE);
		}
		mat[x][y+1] = MARK_EXPLORED;
	}
}

/*###############################################################*/
 void FindPath(point p1, point p2) {

	std::vector<int> trail;
	mat[p1.x][p1.y] = MARK_PATH;
	NextMove(p1, p2, 0, trail);
	
	while(nodeQ.size() > 0) {
		node nn = nodeQ.front();

		if(nn.p.x==p2.x && nn.p.y==p2.y && nn.heuristic < goalDistance) {
			std::cout << nn.p.x << "," << nn.p.y << " ------>\n";
			trail = nn.trail;
			goalDistance = nn.heuristic;
		}
		nodeQ.erase(nodeQ.begin());
		NextMove(nn.p, p2, nn.traveled, nn.trail);
	}

	for(int i=0; i<trail.size(); i++){
		if(trail[i] == LEFT_MOVE) {
			p1.x = p1.x-1;
		} else if(trail[i]==RIGHT_MOVE) {
			p1.x = p1.x+1;
		} else if(trail[i]==UP_MOVE) {
			p1.y = p1.y-1;
		} else if(trail[i]==DOWN_MOVE) {
			p1.y = p1.y+1;
		} else {
			std::cout << "Wrong move\n";
		} 
		mat[p1.x][p1.y] = MARK_PATH;
	}
	
	mat[p2.x][p2.y] = MARK_PATH;
}

/*###############################################################*/
int main() {

	int blocks = 43;
	int points[][2] = {6,4,12,4,18,4,24,4,32,4,38,4,44,4,
				   6,10,12,10,18,10,24,10,32,10,38,10,44,10,
				   6,18,12,18,18,18,24,18,32,18,38,18,44,18,
				   6,28,12,28,18,28,24,28,32,28,38,28,44,28,
				   6,36,12,36,18,36,24,36,32,36,38,36,44,36,
				   6,42,12,42,18,42,24,42,32,42,38,42,44,42, 
					10,0	};
				   
	int height[] = {4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
	int width[] = {4,4,4,4,4,4,4,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
	
	/**
	 * generate floor-plan as a matrix
	 */
	/* fill the whole area with zeros, i.e., empty space */
	for(int i=0; i<AREA_HEIGHT; i++) {
		for(int j=0; j<AREA_WIDTH; j++) {
			mat[i][j] = MARK_EMPTY;
		}
	}
	/* fill the blocks with 1 signifying obstacles */	
	for(int b=0; b<blocks; b++){
		int x = points[b][0];
		int y = points[b][1];
		int w = width[b];
		int h = height[b];

		for(int i=0; i<h; i++) {
			for(int j=0; j<w; j++) {
				mat[x+i][y+j] = MARK_OBSTACKLE;
			}
		}
	}
	

	/**
	 * specify source and destination point,
	 * and find the path between them
	 */
	point p1, p2;
	p1.x = 20; p1.y = 49;
	p2.x = 9; p2.y = 0;

	FindPath(p1, p2);


	/**
	 * print the floor plan
	 */
	for(int i=0; i<AREA_HEIGHT; i++) {
		for(int j=0; j<AREA_WIDTH; j++) {
			if(mat[i][j] == MARK_EMPTY || mat[i][j] == MARK_EXPLORED) {
				std::cout << ".";
			} else if(mat[i][j] == MARK_PATH) {
				std::cout << "x";
			} else {
				std::cout << mat[i][j];
			}
		}
		std::cout << "\n";
	}

	return 0;
}
