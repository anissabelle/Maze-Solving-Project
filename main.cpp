#include <iostream>
#include <fstream>
#include <map>
#include <list>
#include <string>
#include <vector>
#include <queue>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <limits>
#include <filesystem>
using namespace std;
namespace fs = std::filesystem;

// Using the chrono library: https://kahimyang.com/developer/3146/exploring-c-stdchrono-with-comprehensive-examples for elapsed time of algorithms

// https://www.geeksforgeeks.org/cpp/implementation-of-graph-in-cpp/
class Graph{
    map<int, list<int> > adjList;
};

// DFS Algorithm: https://www.geeksforgeeks.org/dsa/depth-first-search-or-dfs-for-a-graph/
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

// BFS Algorithm: https://www.geeksforgeeks.org/dsa/breadth-first-search-or-bfs-for-a-graph/
vector<int> bfs(vector<vector<int>> &adj){
    int V = adj.size();
    vector<bool> visited(V,false);
    vector<int> res;

    queue<int> q;

    int src = 0;
    visited[src] = true;
    q.push(src);

    while(!q.empty()){
        int curr = q.front();
        q.pop();
        res.push_back(curr);

        for(int x : adj[curr]){
            if(!visited[x]){
                visited[x] = true;
                q.push(x);
            }
        }
    }

    return res;

}

vector<string> loadMaze(const string& path) {
    ifstream in(path);
    if (!in) return {};
    vector<string> grid;
    string line;
    while (getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) grid.push_back(line);
    }
    if (grid.empty()) return {};
    const size_t w = grid.front().size();
    for (const auto& row : grid) {
        if (row.size() != w) return {};
        for (char ch : row) {
            if (ch != '#' && ch != '.') return {};
        }
    }
    return grid;
}

vector<vector<int>> buildAdjFromMaze(const vector<string>& grid) {
    const int h = static_cast<int>(grid.size());
    const int w = static_cast<int>(grid[0].size());
    auto id = [w](int r, int c) { return r * w + c; };
    vector<vector<int>> adj(h * w);
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (grid[r][c] == '#') continue;
            const int u = id(r, c);
            for (int k = 0; k < 4; ++k) {
                const int nr = r + dr[k];
                const int nc = c + dc[k];
                if (nr >= 0 && nr < h && nc >= 0 && nc < w && grid[nr][nc] == '.') {
                    adj[u].push_back(id(nr, nc));
                }
            }
        }
    }
    return adj;
}

int firstOpenNode(const vector<string>& grid) {
    const int h = static_cast<int>(grid.size());
    const int w = static_cast<int>(grid[0].size());
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (grid[r][c] == '.') return r * w + c;
        }
    }
    return -1;
}

int lastOpenNode(const vector<string>& grid) {
    const int h = static_cast<int>(grid.size());
    const int w = static_cast<int>(grid[0].size());
    for (int r = h - 1; r >= 0; --r) {
        for (int c = w - 1; c >= 0; --c) {
            if (grid[r][c] == '.') return r * w + c;
        }
    }
    return -1;
}

vector<int> Astar(const vector<string>& grid, int src, int goal){
    vector<int> path;
    const int h = static_cast<int>(grid.size());
    const int w = static_cast<int>(grid[0].size());
    const int n = h * w;
    if (src < 0 || goal < 0 || src >= n || goal >= n) return path;

    auto rc = [w](int id) { return pair<int, int>{id / w, id % w}; };
    auto heuristic = [&](int a, int b) {
        auto [ar, ac] = rc(a);
        auto [br, bc] = rc(b);
        return abs(ar - br) + abs(ac - bc);
    };

    vector<int> g(n, numeric_limits<int>::max());
    vector<int> parent(n, -1);
    vector<char> closed(n, 0);
    using Node = pair<int, int>;
    priority_queue<Node, vector<Node>, greater<Node>> open;

    g[src] = 0;
    open.push({heuristic(src, goal), src});
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    while (!open.empty()) {
        int u = open.top().second;
        open.pop();
        if (closed[u]) continue;
        closed[u] = 1;
        if (u == goal) break;

        auto [r, c] = rc(u);
        for (int k = 0; k < 4; ++k) {
            int nr = r + dr[k];
            int nc = c + dc[k];
            if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
            if (grid[nr][nc] == '#') continue;
            int v = nr * w + nc;
            if (closed[v]) continue;
            int tentative = g[u] + 1;
            if (tentative < g[v]) {
                g[v] = tentative;
                parent[v] = u;
                open.push({tentative + heuristic(v, goal), v});
            }
        }
    }

    if (src == goal) {
        path.push_back(src);
        return path;
    }
    if (parent[goal] == -1) return path;
    for (int at = goal; at != -1; at = parent[at]) path.push_back(at);
    reverse(path.begin(), path.end());
    return path;
}

vector<int> Dijkstra(const vector<string>& grid, int src, int goal) {
    vector<int> path;
    const int h = static_cast<int>(grid.size());
    const int w = static_cast<int>(grid[0].size());
    const int n = h * w;
    if (src < 0 || goal < 0 || src >= n || goal >= n) return path;

    vector<int> dist(n, numeric_limits<int>::max());
    vector<int> parent(n, -1);
    vector<char> done(n, 0);
    using Node = pair<int, int>;
    priority_queue<Node, vector<Node>, greater<Node>> pq;

    dist[src] = 0;
    pq.push({0, src});
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (done[u]) continue;
        done[u] = 1;
        if (u == goal) break;

        int r = u / w;
        int c = u % w;
        for (int k = 0; k < 4; ++k) {
            int nr = r + dr[k];
            int nc = c + dc[k];
            if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
            if (grid[nr][nc] == '#') continue;
            int v = nr * w + nc;
            int nd = dist[u] + 1;
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                pq.push({nd, v});
            }
        }
    }

    if (src == goal) {
        path.push_back(src);
        return path;
    }
    if (parent[goal] == -1) return path;
    for (int at = goal; at != -1; at = parent[at]) path.push_back(at);
    reverse(path.begin(), path.end());
    return path;
}

// Ant Colony Optimization: https://projectsinventory.com/simulation-of-ant-colony-optimization-gaming-project-in-c/

// Grid Dimensions
const int WIDTH = 10;
const int HEIGHT = 10;

// Directions for movement (right, down, left, up)
const int dx[] = {0, 1, 0, -1};
const int dy[] = {1, 0, -1, 0};

class Ant{
    public:
        int x, y;
        Ant(int startX, int startY) : x(startX), y(startY) {}
        void move(){
            int direction = rand() % 4;
            x += dx[direction];
            y += dy[direction];
            x = max(0, min(WIDTH - 1, x));
            y = max(0, min(HEIGHT - 1, y));

        }
};

void AntColonyOpt(vector<Ant> ants, int numAnts, bool verbose = false){
    int startX = 0, startY = 0;
    int goalX = WIDTH - 1, goalY = HEIGHT - 1;
 
    // Initialize ants
    for (int i = 0; i < numAnts; ++i) {
        ants.emplace_back(startX, startY);
    }
 
    for (int step = 0; step < 100; ++step) {
        for (auto& ant : ants) {
            ant.move();
        }

        bool goalReached = false;
        for (const auto& ant : ants) {
            if (ant.x == goalX && ant.y == goalY) {
                goalReached = true;
                break;
            }
        }

        if (goalReached) {
            if (verbose) cout << "An ant has reached the goal!" << endl;
            break;
        }
    }
} 

struct FileMeta {
    string type = "unknown";
    string perfection = "unknown";
    string size = "unknown";
    string index = "unknown";
};

FileMeta parseFileMeta(const fs::path& p) {
    FileMeta meta;
    string stem = p.stem().string();
    vector<string> parts;
    size_t start = 0;
    while (true) {
        size_t pos = stem.find('_', start);
        if (pos == string::npos) {
            parts.push_back(stem.substr(start));
            break;
        }
        parts.push_back(stem.substr(start, pos - start));
        start = pos + 1;
    }
    if (parts.size() >= 1) {
        meta.type = parts[0];
        meta.perfection = parts[0];
    }
    if (parts.size() >= 2) meta.size = parts[1];
    if (parts.size() >= 3) meta.index = parts[2];
    return meta;
}

struct Timings {
    long long dfsUs = 0;
    long long bfsUs = 0;
    long long astarUs = 0;
    long long dijkstraUs = 0;
    long long antUs = 0;
};

Timings runAllTimings(const vector<string>& maze) {
    Timings t;
    vector<vector<int>> adj = buildAdjFromMaze(maze);
    int src = firstOpenNode(maze);
    int goal = lastOpenNode(maze);
    if (src < 0) return t;
    if (goal < 0) goal = src;

    {
        auto t0 = chrono::steady_clock::now();
        vector<bool> visited(adj.size(), false);
        vector<int> res;
        dfsRec(adj, visited, src, res);
        auto t1 = chrono::steady_clock::now();
        t.dfsUs = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
    }
    {
        auto t0 = chrono::steady_clock::now();
        vector<bool> visited(adj.size(), false);
        vector<int> res;
        queue<int> q;
        visited[src] = true;
        q.push(src);
        while(!q.empty()){
            int curr = q.front();
            q.pop();
            res.push_back(curr);
            for(int x : adj[curr]){
                if(!visited[x]){
                    visited[x] = true;
                    q.push(x);
                }
            }
        }
        auto t1 = chrono::steady_clock::now();
        t.bfsUs = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
    }
    {
        auto t0 = chrono::steady_clock::now();
        auto res = Astar(maze, src, goal);
        (void)res;
        auto t1 = chrono::steady_clock::now();
        t.astarUs = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
    }
    {
        auto t0 = chrono::steady_clock::now();
        auto res = Dijkstra(maze, src, goal);
        (void)res;
        auto t1 = chrono::steady_clock::now();
        t.dijkstraUs = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
    }
    {
        auto t0 = chrono::steady_clock::now();
        vector<Ant> ants;
        AntColonyOpt(ants, 10, false);
        auto t1 = chrono::steady_clock::now();
        t.antUs = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
    }
    return t;
}

void runDirectoryBatch(const fs::path& dir) {
    vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            files.push_back(entry.path());
        }
    }
    sort(files.begin(), files.end());
    if (files.empty()) {
        cout << "No .txt maze files found in directory." << endl;
        return;
    }

    for (const auto& file : files) {
        vector<string> maze = loadMaze(file.string());
        if (maze.empty()) {
            cout << file.filename().string() << " -> parse failed" << endl;
            continue;
        }
        FileMeta meta = parseFileMeta(file);
        Timings t = runAllTimings(maze);
        cout << file.filename().string()
             << " | type: " << meta.type
             << " | perfection: " << meta.perfection
             << " | size: " << meta.size
             << " | time(us) dfs=" << t.dfsUs
             << " bfs=" << t.bfsUs 
             << " astar=" << t.astarUs
             << " dijkstra=" << t.dijkstraUs
             << " aco=" << t.antUs
             << endl;
    }
}

int main(int argc, char** argv){

    int numAnts = 0;
    int choice = 0;
    string datasetPath;
    if (argc > 1) {
        datasetPath = argv[1];
    } else {
        cout << "Dataset file path: ";
        getline(cin, datasetPath);
    }

    fs::path inputPath(datasetPath);
    if (fs::exists(inputPath) && fs::is_directory(inputPath)) {
        runDirectoryBatch(inputPath);
        return 0;
    }

    vector<string> maze = loadMaze(datasetPath);
    if (maze.empty()) {
        cerr << "Could not parse maze file." << endl;
        return 1;
    }
    vector<vector<int>> adj = buildAdjFromMaze(maze);
    int src = firstOpenNode(maze);
    int goal = lastOpenNode(maze);
    if (src < 0) {
        cerr << "No open cells in maze." << endl;
        return 1;
    }
    if (goal < 0) goal = src;

    // Displaying Menu
    cout << "*********  Algorithm Times  *********" << endl;
    cout << "1. DFS" << endl;
    cout << "2. BFS" << endl;
    cout << "3. A*" << endl;
    cout << "4. Dijkstra" << endl;
    cout << "5. Ant Colony Optimization" << endl;
    cout << "6. Compare all results" << endl << endl;
    cout << "7. Exit" << endl;
    cout << "Select 1-7: " << endl;
    cin >> choice;
    while(choice < 1 || choice > 7){
        cout << "You must enter a number 1-7" << endl;
        cin >> choice;
    }

    switch(choice){
        case 1: {
            // DFS
            auto t0 = std::chrono::steady_clock::now();
            auto dfsRes = dfs(adj);
            if (!dfsRes.empty() && dfsRes[0] != src) {
                vector<bool> visited(adj.size(), false);
                dfsRes.clear();
                dfsRec(adj, visited, src, dfsRes);
            }

            auto t1 = std::chrono::steady_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
            cout << "Execution time of DFS traversal: " << us.count() << endl;
        break;
        }

        case 2: {
            // BFS
            auto t0 = std::chrono::steady_clock::now();
            vector<int> bfsRes;
            vector<bool> visited(adj.size(), false);
            queue<int> q;
            visited[src] = true;
            q.push(src);
            while(!q.empty()){
                int curr = q.front();
                q.pop();
                bfsRes.push_back(curr);
                for(int x : adj[curr]){
                    if(!visited[x]){
                        visited[x] = true;
                        q.push(x);
                    }
                }
            }

            auto t1 = std::chrono::steady_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
            cout << "Execution time of BFS traversal: " << us.count() << endl;
        break;
        }

        case 3: {
            // A*
            auto t0 = std::chrono::steady_clock::now();
            auto astarRes = Astar(maze, src, goal);

            auto t1 = std::chrono::steady_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
            cout << "Execution time of A*: " << us.count() << endl;
            cout << "A* path length: " << astarRes.size() << endl;
        break;
        }

        case 4: {
            auto t0 = std::chrono::steady_clock::now();
            auto dijkstraRes = Dijkstra(maze, src, goal);
            auto t1 = std::chrono::steady_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
            cout << "Execution time of Dijkstra: " << us.count() << endl;
            cout << "Dijkstra path length: " << dijkstraRes.size() << endl;
        break;
        }

        case 5: {
            // Ant Colony Optimization
            cout << "How many ants are solving the maze?" << endl;
            cout << "Number of Ants: ";
            cin >> numAnts;
            auto t0 = std::chrono::steady_clock::now();
            vector<Ant> ants;
            AntColonyOpt(ants, numAnts, false);

            auto t1 = std::chrono::steady_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
            cout << "Execution time of Ant Colony Optimization: " << us.count() << endl;
        break;
        }

        case 6: {
            auto dfsStart = std::chrono::steady_clock::now();
            vector<bool> dfsVisited(adj.size(), false);
            vector<int> dfsRes;
            dfsRec(adj, dfsVisited, src, dfsRes);
            auto dfsEnd = std::chrono::steady_clock::now();
            auto dfsUs = std::chrono::duration_cast<std::chrono::microseconds>(dfsEnd - dfsStart);
            cout << "Execution time of DFS traversal: " << dfsUs.count() << endl;

            auto bfsStart = std::chrono::steady_clock::now();
            vector<bool> bfsVisited(adj.size(), false);
            vector<int> bfsRes;
            queue<int> q;
            bfsVisited[src] = true;
            q.push(src);
            while(!q.empty()){
                int curr = q.front();
                q.pop();
                bfsRes.push_back(curr);
                for(int x : adj[curr]){
                    if(!bfsVisited[x]){
                        bfsVisited[x] = true;
                        q.push(x);
                    }
                }
            }
            auto bfsEnd = std::chrono::steady_clock::now();
            auto bfsUs = std::chrono::duration_cast<std::chrono::microseconds>(bfsEnd - bfsStart);
            cout << "Execution time of BFS traversal: " << bfsUs.count() << endl;

            auto astarStart = std::chrono::steady_clock::now();
            auto astarRes = Astar(maze, src, goal);
            auto astarEnd = std::chrono::steady_clock::now();
            auto astarUs = std::chrono::duration_cast<std::chrono::microseconds>(astarEnd - astarStart);
            cout << "Execution time of A*: " << astarUs.count() << endl;

            auto dijkstraStart = std::chrono::steady_clock::now();
            auto dijkstraRes = Dijkstra(maze, src, goal);
            auto dijkstraEnd = std::chrono::steady_clock::now();
            auto dijkstraUs = std::chrono::duration_cast<std::chrono::microseconds>(dijkstraEnd - dijkstraStart);
            cout << "Execution time of Dijkstra: " << dijkstraUs.count() << endl;

            auto antStart = std::chrono::steady_clock::now();
            vector<Ant> ants;
            AntColonyOpt(ants, numAnts > 0 ? numAnts : 10, false);
            auto antEnd = std::chrono::steady_clock::now();
            auto antUs = std::chrono::duration_cast<std::chrono::microseconds>(antEnd - antStart);
            cout << "Execution time of Ant Colony Optimization: " << antUs.count() << endl;
        break;
        }

        case 7:
            cout << "Goodbye!" << endl;
            return 0;
        break;
    }
}

