/**
 * \file mapp.cpp (makespan-minimized multi-agent path planning)
 * 
 * \author Aritra Sarkar
 * \date 11-09-2017 (begin)
 */
 
/** References
 * 
 */

// Agent Code

#include <vector>
#include <iostream>

const int NUM_U = 3;
const int NUM_V = 3;
const int INF = 10000;
const int NIL = NUM_U;	// Dummy Node Number
	
class HopcroftKarp
{
	
public:
	
	int Adj[NUM_U][NUM_V];
	int Pair_U[NUM_U];	
	int Pair_V[NUM_V];

private:

	int Dist[NUM_U+1];
	std::vector<int> Q;

	// ############################################## @$ ##############################################
	
public:

	int maxmatch()
	{
		int u, v;
		for (u = 0; u < NUM_U; u++)
			Pair_U[u] = NIL;
		for (v = 0; v < NUM_V; v++)
			Pair_V[v] = NIL;
		int matching = 0;
		while (BFS()) {
			for (u = 0; u < NUM_U; u++) {
				if (Pair_U[u] == NIL && DFS(u))
					matching++;
			}
		}
		return matching;
	}

	// ############################################## @$ ##############################################

private:
	
	bool BFS()
	{
		int u, v;
		for (u = 0; u < NUM_U; u++) {
			if (Pair_U[u] == NIL) {
				Dist[u] = 0;
				Q.push_back(u);
			}
			else {
				Dist[u] = INF;
			}
		}
		Dist[NIL] = INF;
		while (Q.size() > 0) {
			u = Q.front();	
			Q.erase(Q.begin());
			if (Dist[u] < Dist[NIL]) {
				for (v = 0; v < NUM_V; v++) {
					if (Adj[u][v] == 0)
						continue;
					if (Dist[Pair_V[v]] == INF) {
						Dist[Pair_V[v]] = Dist[u] + 1;
						Q.push_back(Pair_V[v]);
					}
				}
			}
		}
		return (Dist[NIL] != INF);
	}
	
	// ############################################## @$ ##############################################
	
	bool DFS(int u)
	{
		int v;
		if (u != NIL) {
			for (v = 0; v < NUM_V; v++) {
				if (Adj[u][v] == 0)
					continue;
				if (Dist[Pair_V[v]] == Dist[u] + 1) {
					if (DFS(Pair_V[v])) {
						Pair_V[v] = u;
						Pair_U[u] = v;
						return true;
					}
				}
			}
			Dist[u] = INF;
			return false;	
		}
		return true;	
	}
	
};

	// ############################################## @$ ##############################################

class MAPP
{
	
public:

	int BPG[NUM_U][NUM_V];
	
	MAPP()
	{
		BPG[0][0] = 1;
		BPG[0][1] = 16;
		BPG[0][2] = 10;
		BPG[1][0] = 2;
		BPG[1][1] = 20;
		BPG[1][2] = 15;
		BPG[2][0] = 3;
		BPG[2][1] = 30;
		BPG[2][2] = 30;
	}
};

	// ############################################## @$ ##############################################

	int main(int argc, char** argv)
	{
		HopcroftKarp hck;
		MAPP mm;
		int u, v, max = 0, min = INF, mkspn;
		bool found = false;
		for (u = 0; u < NUM_U; u++) {
			for (v = 0; v < NUM_V; v++) {
				if (mm.BPG[u][v] < min)
					min = mm.BPG[u][v];
				if (mm.BPG[u][v] > max)
					max = mm.BPG[u][v];
			}
		}
		
		for (mkspn = min; mkspn <= max; mkspn++) {
			for (u = 0; u < NUM_U; u++) {
				for (v = 0; v < NUM_V; v++) {
					if (mm.BPG[u][v] <= mkspn)
						hck.Adj[u][v] = 1;
					else
						hck.Adj[u][v] = 0;
				}
			}
			if (hck.maxmatch() == NUM_U) {
				if (found == false) {
					std::cout << "\nMaximum Matching with Minimum Makespan" << std::endl;
					for (u = 0; u < NUM_U; u++)
						std::cout << "u = " << u << "\tv = " << hck.Pair_U[u] << std::endl;
					std::cout << std::endl;
					found = true;
				}
				std::cout << "Hopcroft Karp success for makespan = " << mkspn << std::endl;
			}
			else
				std::cout << "Hopcroft Karp failure for makespan = " << mkspn << std::endl;
		}
	}
	
	// ############################################## @$ ##############################################
