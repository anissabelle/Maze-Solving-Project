#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

vector<string> loadGrid(const string& path) {
    ifstream in(path);
    vector<string> grid;
    string line;
    while (getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) grid.push_back(line);
    }
    return grid;
}

vector<vector<int>> buildAdj(const vector<string>& g) {
    int h = static_cast<int>(g.size());
    int w = static_cast<int>(g[0].size());
    auto id = [w](int r, int c) { return r * w + c; };
    vector<vector<int>> adj(h * w);
    int dr[4] = {-1, 1, 0, 0};
    int dc[4] = {0, 0, -1, 1};
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (g[r][c] == '#') continue;
            int u = id(r, c);
            for (int k = 0; k < 4; ++k) {
                int nr = r + dr[k], nc = c + dc[k];
                if (nr >= 0 && nr < h && nc >= 0 && nc < w && g[nr][nc] == '.') adj[u].push_back(id(nr, nc));
            }
        }
    }
    return adj;
}

pair<int, int> srcGoal(const vector<string>& g) {
    int h = static_cast<int>(g.size());
    int w = static_cast<int>(g[0].size());
    int src = -1, goal = -1;
    for (int r = 0; r < h && src == -1; ++r) for (int c = 0; c < w; ++c) if (g[r][c] == '.') { src = r * w + c; break; }
    for (int r = h - 1; r >= 0 && goal == -1; --r) for (int c = w - 1; c >= 0; --c) if (g[r][c] == '.') { goal = r * w + c; break; }
    if (goal == -1) goal = src;
    return {src, goal};
}

template <class F>
long long us(F&& f) {
    auto t0 = chrono::steady_clock::now();
    f();
    auto t1 = chrono::steady_clock::now();
    return chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
}

int runDfs(const vector<vector<int>>& adj, int src) {
    if (src < 0) return 0;
    vector<char> vis(adj.size(), 0);
    vector<int> st{src};
    int seen = 0;
    while (!st.empty()) {
        int u = st.back();
        st.pop_back();
        if (vis[u]) continue;
        vis[u] = 1;
        ++seen;
        for (int v : adj[u]) if (!vis[v]) st.push_back(v);
    }
    return seen;
}

int runBfs(const vector<vector<int>>& adj, int src) {
    if (src < 0) return 0;
    vector<char> vis(adj.size(), 0);
    queue<int> q;
    q.push(src);
    vis[src] = 1;
    int seen = 0;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        ++seen;
        for (int v : adj[u]) if (!vis[v]) { vis[v] = 1; q.push(v); }
    }
    return seen;
}

vector<int> Astar(const vector<string>& g, int src, int goal) {
    vector<int> path;
    int h = static_cast<int>(g.size()), w = static_cast<int>(g[0].size()), n = h * w;
    if (src < 0 || goal < 0 || src >= n || goal >= n) return path;
    auto rc = [w](int id) { return pair<int, int>{id / w, id % w}; };
    auto h1 = [&](int a, int b) { auto [ar, ac] = rc(a); auto [br, bc] = rc(b); return abs(ar - br) + abs(ac - bc); };
    vector<int> dist(n, numeric_limits<int>::max()), parent(n, -1);
    vector<char> closed(n, 0);
    using Node = pair<int, int>;
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    int dr[4] = {-1, 1, 0, 0}, dc[4] = {0, 0, -1, 1};
    dist[src] = 0;
    pq.push({h1(src, goal), src});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (closed[u]) continue;
        closed[u] = 1;
        if (u == goal) break;
        auto [r, c] = rc(u);
        for (int k = 0; k < 4; ++k) {
            int nr = r + dr[k], nc = c + dc[k];
            if (nr < 0 || nr >= h || nc < 0 || nc >= w || g[nr][nc] == '#') continue;
            int v = nr * w + nc, nd = dist[u] + 1;
            if (nd < dist[v]) { dist[v] = nd; parent[v] = u; pq.push({nd + h1(v, goal), v}); }
        }
    }
    if (src == goal) return {src};
    if (parent[goal] == -1) return path;
    for (int at = goal; at != -1; at = parent[at]) path.push_back(at);
    reverse(path.begin(), path.end());
    return path;
}

vector<int> Dijkstra(const vector<string>& g, int src, int goal) {
    vector<int> path;
    int h = static_cast<int>(g.size()), w = static_cast<int>(g[0].size()), n = h * w;
    if (src < 0 || goal < 0 || src >= n || goal >= n) return path;
    vector<int> dist(n, numeric_limits<int>::max()), parent(n, -1);
    vector<char> done(n, 0);
    using Node = pair<int, int>;
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    int dr[4] = {-1, 1, 0, 0}, dc[4] = {0, 0, -1, 1};
    dist[src] = 0;
    pq.push({0, src});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (done[u]) continue;
        done[u] = 1;
        if (u == goal) break;
        int r = u / w, c = u % w;
        for (int k = 0; k < 4; ++k) {
            int nr = r + dr[k], nc = c + dc[k];
            if (nr < 0 || nr >= h || nc < 0 || nc >= w || g[nr][nc] == '#') continue;
            int v = nr * w + nc, nd = dist[u] + 1;
            if (nd < dist[v]) { dist[v] = nd; parent[v] = u; pq.push({nd, v}); }
        }
    }
    if (src == goal) return {src};
    if (parent[goal] == -1) return path;
    for (int at = goal; at != -1; at = parent[at]) path.push_back(at);
    reverse(path.begin(), path.end());
    return path;
}

vector<string> parseStem(const fs::path& file) {
    string s = file.stem().string();
    vector<string> out;
    size_t start = 0;
    while (true) {
        size_t pos = s.find('_', start);
        if (pos == string::npos) { out.push_back(s.substr(start)); break; }
        out.push_back(s.substr(start, pos - start));
        start = pos + 1;
    }
    return out;
}

void runDirectory(const fs::path& dir) {
    vector<fs::path> files;
    for (const auto& e : fs::directory_iterator(dir)) if (e.is_regular_file() && e.path().extension() == ".txt") files.push_back(e.path());
    sort(files.begin(), files.end());
    for (const auto& file : files) {
        auto maze = loadGrid(file.string());
        if (maze.empty()) continue;
        auto adj = buildAdj(maze);
        auto sg = srcGoal(maze);
        int src = sg.first;
        int goal = sg.second;
        auto name = parseStem(file);
        string type = name.size() > 0 ? name[0] : "unknown";
        string size = name.size() > 1 ? name[1] : "unknown";
        long long dfsUs = us([&] { (void)runDfs(adj, src); });
        long long bfsUs = us([&] { (void)runBfs(adj, src); });
        long long astarUs = us([&] { (void)Astar(maze, src, goal); });
        long long dijkstraUs = us([&] { (void)Dijkstra(maze, src, goal); });
        cout << file.filename().string()
             << " | perfection: " << type
             << " | size: " << size
             << " | time(us) dfs=" << dfsUs
             << " bfs=" << bfsUs
             << " astar=" << astarUs
             << " dijkstra=" << dijkstraUs
             << '\n';
    }
}

void runMenu(const vector<string>& maze) {
    auto adj = buildAdj(maze);
    auto sg = srcGoal(maze);
    int src = sg.first;
    int goal = sg.second;
    cout << "1. DFS\n2. BFS\n3. A*\n4. Dijkstra\n5. Compare all\n6. Exit\nSelect 1-6: ";
    int choice = 0;
    cin >> choice;
    if (choice == 1) {
        cout << "Execution time of DFS traversal: " << us([&] { (void)runDfs(adj, src); }) << '\n';
    } else if (choice == 2) {
        cout << "Execution time of BFS traversal: " << us([&] { (void)runBfs(adj, src); }) << '\n';
    } else if (choice == 3) {
        vector<int> path;
        long long t = us([&] { path = Astar(maze, src, goal); });
        cout << "Execution time of A*: " << t << '\n';
        cout << "A* path length: " << path.size() << '\n';
    } else if (choice == 4) {
        vector<int> path;
        long long t = us([&] { path = Dijkstra(maze, src, goal); });
        cout << "Execution time of Dijkstra: " << t << '\n';
        cout << "Dijkstra path length: " << path.size() << '\n';
    } else if (choice == 5) {
        cout << "Execution time of DFS traversal: " << us([&] { (void)runDfs(adj, src); }) << '\n';
        cout << "Execution time of BFS traversal: " << us([&] { (void)runBfs(adj, src); }) << '\n';
        cout << "Execution time of A*: " << us([&] { (void)Astar(maze, src, goal); }) << '\n';
        cout << "Execution time of Dijkstra: " << us([&] { (void)Dijkstra(maze, src, goal); }) << '\n';
    }
}

int main(int argc, char** argv) {
    string input = argc > 1 ? argv[1] : "";
    if (input.empty()) { cout << "Dataset file path: "; getline(cin, input); }
    fs::path p(input);
    if (fs::exists(p) && fs::is_directory(p)) {
        runDirectory(p);
        return 0;
    }
    auto maze = loadGrid(input);
    if (maze.empty()) return 1;
    runMenu(maze);
    return 0;
}

