/**
 * \file assign.cpp
 * 
 * \author Aritra Sarkar
 * \date 05-09-2017 (begin)
 */
 
// References
// https://stackoverflow.com/questions/21951100/minimize-maximum-cost?rq=1
// 

// Agent Code

#include <stdlib.h>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>

class assignment
{
	int NUM_U = 3;
	int NUM_V = 3;
	int Adj[NUM_U][NUM_V];
	int Dist[NUM_U][NUM_V];
	int Pair_U[NUM_U];
	int Pair_V[NUM_V];
	int NIL = NUM_U + NUM_V;	// Dummy Node Number
	int INF = 10000;
	public void assignAgt()
	{
		int i, j;
		for (i = 0; i < NUM_U; i++)
			for (j = 0; j < NUM_V; j++)
				Adj[i][j] = 1;
				
		Dist[0][0] = 1;
		Dist[0][1] = 16;
		Dist[0][2] = 10;
		Dist[1][0] = 2;
		Dist[1][1] = 20;
		Dist[1][2] = 15;
		Dist[2][0] = 3;
		Dist[2][1] = 30;
		Dist[2][2] = 30;
		
	}
	public int hopcroftKarp()
	{
		int agt, obj;
		for (agt = 0; agt < NUM_U; agt++)
			Pair_U[agt] = NIL;
		for (obj = 0; obj < NUM_V; obj++)
			Pair_O[obj] = NIL;
		matching = 0;
		while (BFS()) {
			for (agt = 0; agt < NUM_U; agt++) {
				if (Pair_U[agt] = 0 && DFS(agt))
						matching++;
			}
		}
		return matching
	}
	public void BFS()
	{
	
/*
    for each u in U
        if Pair_U[u] == NIL
            Dist[u] = 0
            Enqueue(Q,u)
        else
            Dist[u] = ∞
    Dist[NIL] = ∞
    while Empty(Q) == false
        u = Dequeue(Q)
        if Dist[u] < Dist[NIL] 
            for each v in Adj[u]
                if Dist[ Pair_V[v] ] == ∞
                    Dist[ Pair_V[v] ] = Dist[u] + 1
                    Enqueue(Q,Pair_V[v])
    return Dist[NIL] != ∞
*/	
		
	}
	
	public bool DFS(u)
	{
		if (u == NIL)
			return true;
		for (v = 0; v < NUM_V; v++) {
			if (Adj[u][v] == 0)
				continue;
			if (Dist[Pair_V[v]][v] == Dist[u][v] + 1) {
				if (DFS(Pair_V[v]) {
					Pair_V[v] = u;
					Pair_U[u] = v;
					return true;
				}
			}
		}
		Dist[u][v] = INF;
		return false;		
	}
	
}

/* 
 G = U ∪ V ∪ {NIL}
 where U and V are partition of graph and NIL is a special null vertex
*/

/* 
function BFS ()
    for each u in U
        if Pair_U[u] == NIL
            Dist[u] = 0
            Enqueue(Q,u)
        else
            Dist[u] = ∞
    Dist[NIL] = ∞
    while Empty(Q) == false
        u = Dequeue(Q)
        if Dist[u] < Dist[NIL] 
            for each v in Adj[u]
                if Dist[ Pair_V[v] ] == ∞
                    Dist[ Pair_V[v] ] = Dist[u] + 1
                    Enqueue(Q,Pair_V[v])
    return Dist[NIL] != ∞

function DFS (u)
    if u != NIL
        for each v in Adj[u]
            if Dist[ Pair_V[v] ] == Dist[u] + 1
                if DFS(Pair_V[v]) == true
                    Pair_V[v] = u
                    Pair_U[u] = v
                    return true
        Dist[u] = ∞
        return false
    return true

function Hopcroft-Karp
    for each u in U
        Pair_U[u] = NIL
    for each v in V
        Pair_V[v] = NIL
    matching = 0
    while BFS() == true
        for each u in U
            if Pair_U[u] == NIL
                if DFS(u) == true
                    matching = matching + 1
    return matching
*/
