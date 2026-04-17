using namespace std;
#include <iostream>
#include <map>
#include <list>
#include <vector>


// https://www.geeksforgeeks.org/cpp/implementation-of-graph-in-cpp/
class Graph{
    map<int, list<int> > adjList;
};


// DFS Algorithm
void dfsRec(vector<vector<int>> &adj, vector<bool> &visited, int s, vector<int> &res){

    visited[s] = true;

    res.push_back(s);

    for(int i : adj[s]){
        if(visited[i] == false){
            dfsRec(adj, visited, i, res);
        }
    }
}

vector<int> dfs(vector<vector<int>> &adj){
    vector<bool> visited(adj.size(), false);
    vector<int> res;
    dfsRec(adj, visited, 0, res);
    return res;
}

// BFS Algorithm



// Dikstra




