#include <iostream>
#include <map>
#include <list>
#include <vector>
#include <queue>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <stack>
#include <climits>
#include <cmath>
using namespace std;

// Using the chrono library: https://kahimyang.com/developer/3146/exploring-c-stdchrono-with-comprehensive-examples for elapsed time of algorithms
vector<vector<int>> maze;

struct Point {
    int x, y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const Point& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

bool isValid(int x, int y, const vector<vector<int>>& maze) {
    return y >= 0 && y < maze.size() &&
           x >= 0 && x < maze[0].size() &&
           maze[y][x] == 0;
}

// DFS Algorithm
vector<Point> dfsMaze(const vector<vector<int>>& maze) {
    int rows = maze.size();
    int cols = maze[0].size();

    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    vector<Point> result;
    stack<Point> st;

    st.push({0, 0});

    while (!st.empty()) {
        Point curr = st.top();
        st.pop();

        if (!isValid(curr.x, curr.y, maze) || visited[curr.y][curr.x]) {
            continue;
        }

        visited[curr.y][curr.x] = true;
        result.push_back(curr);

        if (curr.x == cols - 1 && curr.y == rows - 1) {
            break;
        }

        st.push({curr.x + 1, curr.y});
        st.push({curr.x - 1, curr.y});
        st.push({curr.x, curr.y + 1});
        st.push({curr.x, curr.y - 1});
    }

    return result;
}

// BFS Algorithm
vector<Point> bfsMaze(const vector<vector<int>>& maze) {
    int rows = maze.size();
    int cols = maze[0].size();

    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    vector<Point> result;
    queue<Point> q;

    if (!isValid(0, 0, maze)) return result;

    q.push({0, 0});
    visited[0][0] = true;

    while (!q.empty()) {
        Point curr = q.front();
        q.pop();

        result.push_back(curr);

        if (curr.x == cols - 1 && curr.y == rows - 1) {
            break;
        }

        vector<Point> neighbors = { // defines neighboring nodes
            {curr.x + 1, curr.y},
            {curr.x - 1, curr.y},
            {curr.x, curr.y + 1},
            {curr.x, curr.y - 1}
        };

        for (const auto& n : neighbors) {   // Checks what neighbor node paths can be taken
            if (isValid(n.x, n.y, maze) && !visited[n.y][n.x]) {
                visited[n.y][n.x] = true;
                q.push(n);
            }
        }
    }

    return result;
}

// A*
int heuristic(Point a, Point b){
    return abs(a.x - b.x)+ (abs a.y - b.y);
}

vector<Point> Astar(){
    using Node = pair<int, Point>;
    priority_queue<Node, vector<Node>,greater<Node>> pq;
    
    map<Point, int> gCost;
    map<Point, Point> parent;
    
    gCost[start] = 0;
    pq.push({heuristic(start,goal), start});
    
    while (!pq.empty()) {
        Point curr = pq.top().second;
        pq.pop();
        
        if (curr == goal)
          return recontructPath(parent, goal);
        for (auto d : directions) {
            Point next = {curr.x + d.x, curr.y +d.y};
            
            if(!isValid(next.c, next.y)) continue;
            
            int newCost = gCost[curr] + 1;
            
            if(!gCost.count(next) || newCost < gCost[next]) {
                gCost[next] = newCost;
                int priority = newCost + heuristic(next,goal);
                pq.push({priority, next});
                parent[next] = curr;
            }
        }
    }
      return{};
}

// Ant Colony Optimization

// Grid Dimensions
const int WIDTH = 10;
const int HEIGHT = 10;

// Directions for movement (right, down, left, up)
const int dx[] = {0, 1, 0, -1};
const int dy[] = {1, 0, -1, 0};

struct Ant{
        int x, y;
        vector<pair<int,int>> path;
        vector<vector<bool>> visited;
        bool reachedGoal=false;

        Ant(int startX, int startY, int width, int height)
        : x(startX), y(startY), visited(height, vector<bool>(width, false)) {
        path.push_back({x, y});
        visited[y][x] = true;
    }
};


double heuristic(int x, int y, int goalX, int goalY) {
    return 1.0 / (abs(goalX - x) + abs(goalY - y) + 1.0);
}

pair<int,int> chooseNextMove(
    Ant& ant,
    const vector<vector<int>>& maze,
    const vector<vector<double>>& pheromone,
    int goalX, int goalY,
    double alpha, double beta
) {
    vector<pair<int,int>> neighbors = {
        {ant.x + 1, ant.y},
        {ant.x - 1, ant.y},
        {ant.x, ant.y + 1},
        {ant.x, ant.y - 1}
    };

    vector<pair<int,int>> validMoves;
    vector<double> probabilities;
    double total = 0.0;

    for (auto& n : neighbors) {
        int nx = n.first;
        int ny = n.second;

        if (isValid(nx, ny, maze) && !ant.visited[ny][nx]) {
            double tau = pow(pheromone[ny][nx], alpha);
            double eta = pow(heuristic(nx, ny, goalX, goalY), beta);
            double score = tau * eta;

            validMoves.push_back({nx, ny});
            probabilities.push_back(score);
            total += score;
        }
    }

    if (validMoves.empty()) {
        return {-1, -1};
    }

    double r = ((double) rand() / RAND_MAX) * total;
    double cumulative = 0.0;

    for (int i = 0; i < validMoves.size(); i++) {
        cumulative += probabilities[i];
        if (r <= cumulative) {
            return validMoves[i];
        }
    }

    return validMoves.back();
}

vector<pair<int,int>> AntColonyOpt(
    const vector<vector<int>>& maze,
    int numAnts,
    int iterations
) {
    int height = maze.size();
    int width = maze[0].size();

    int startX = 0, startY = 0;
    int goalX = width - 1, goalY = height - 1;

    vector<vector<double>> pheromone(height, vector<double>(width, 1.0));

    double alpha = 1.0;
    double beta = 3.0;
    double evaporation = 0.3;
    double Q = 100.0;
    int maxSteps = width * height;

    vector<pair<int,int>> bestPath;
    int bestLength = INT_MAX;

    srand(time(0));

    for (int iter = 0; iter < iterations; iter++) {
        vector<Ant> ants;

        for (int i = 0; i < numAnts; i++) {
            ants.emplace_back(startX, startY, width, height);
        }

        for (auto& ant : ants) {
            for (int step = 0; step < maxSteps; step++) {
                if (ant.x == goalX && ant.y == goalY) {
                    ant.reachedGoal = true;
                    break;
                }

                pair<int,int> nextMove = chooseNextMove(
                    ant, maze, pheromone, goalX, goalY, alpha, beta
                );

                if (nextMove.first == -1) {
                    break;
                }

                ant.x = nextMove.first;
                ant.y = nextMove.second;
                ant.path.push_back({ant.x, ant.y});
                ant.visited[ant.y][ant.x] = true;
            }

            if (ant.x == goalX && ant.y == goalY) {
                ant.reachedGoal = true;

                if (ant.path.size() < bestLength) {
                    bestLength = ant.path.size();
                    bestPath = ant.path;
                }
            }
        }

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                pheromone[y][x] *= (1.0 - evaporation);
                if (pheromone[y][x] < 0.01) {
                    pheromone[y][x] = 0.01;
                }
            }
        }

        for (const auto& ant : ants) {
            if (ant.reachedGoal) {
                double deposit = Q / ant.path.size();
                for (const auto& cell : ant.path) {
                    int x = cell.first;
                    int y = cell.second;
                    pheromone[y][x] += deposit;
                }
            }
        }

        cout << "Iteration " << iter + 1;
        if (bestLength < INT_MAX) {
            cout << " | Best path length so far: " << bestLength << endl;
        } else {
            cout << " | No path found yet" << endl;
        }
    }

    return bestPath;
}

int main(){

    vector<vector<int>> maze;
    int numAnts = 0;
    int choice = 0;
    int graphSizeChoice = 0;


    // Displaying Menu
    cout << "*********  Algorithm Times  *********" << endl;
    cout << "Select Graph size: " << endl;
    cout << "1. imperfect small" << endl;
    cout << "2. perfect small" << endl;
    cout << "3. imperfect medium" << endl;
    cout << "4. perfect medium" << endl;
    cout << "5. imperfect large" << endl;
    cout << "6. perfect large" << endl << endl;
    cout << "Size: ";
    cin >> graphSizeChoice;

    while(graphSizeChoice < 1 || graphSizeChoice > 6){
        cout << "Please choose a valid graph size: ";
        cin >> graphSizeChoice;
    }

    switch(graphSizeChoice){
        case 1:{
            break;
        }
        case 2:{
            break;
        }
        case 3:{
            break;
        }
        case 4:{
            break;
        }
        case 5:{
            break;
        }
        case 6:{
            break;
        }
    }

    cout << "1. DFS" << endl;
    cout << "2. BFS" << endl;
    cout << "3. A*" << endl;
    cout << "4. Ant Colony Optimization" << endl;
    cout << "5. Compare all results" << endl << endl;
    cout << "6. Exit" << endl;
    cout << "Select 1-5: " << endl;
    cin >> choice;
    while(choice < 1 || choice > 6){
        cout << "You must enter a number 1-6" << endl;
        cin >> choice;
    }

    switch(choice){
        case 1:{
            // DFS
            auto start = std::chrono::steady_clock::now();
            dfsMaze(maze);
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of DFS traversal: " << duration.count() << endl;
        break;
        }
        case 2:{
            // BFS
            auto start = std::chrono::steady_clock::now();
            bfsMaze(maze);
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of BFS traversal: " << duration.count() << endl;
        break;
        }  
        case 3:{
            // A*
            auto start = std::chrono::steady_clock::now();
            Astar(maze);
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of A*: " << duration.count() << endl;
        break;
        }
        case 4:{
            // Ant Colony Optimization
            cout << "How many ants are solving the maze?" << endl;
            cout << "Number of Ants: ";
            cin >> numAnts;
            auto start = std::chrono::steady_clock::now();
            AntColonyOpt(maze);
            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of Ant Colony Optimization: " << duration.count() << endl;
        break;
        }
        case 5:{
            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of DFS traversal: " << duration.count() << endl;

            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of BFS traversal: " << duration.count() << endl;

            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of A*: " << duration.count() << endl;

            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of Ant Colony Optimization: " << duration.count() << endl;
        break;
        }
        case 6:{
            cout << "Goodbye!" << endl;
            return 0;
        break;
        }
    }
}

