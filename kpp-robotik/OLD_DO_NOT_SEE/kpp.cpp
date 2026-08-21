#include <bits/stdc++.h>
using namespace std;

const int MAX_ENERGY = 1000;

struct Edge {
    string to;
    int w, o;
};

class Graph {
public:
    int n;
    unordered_map<string, vector<Edge>> adj;

    Graph(int n) : n(n) {}

    void add_edge(const string &u, const string &v, int w, int o) {
        adj[u].push_back({v, w, o});
        adj[v].push_back({u, w, o});
    }
};

struct State {
    int energy_used;
    int time;
    string node;
    int energy_left;
    vector<string> path;

    bool operator>(const State &other) const {
        return energy_used > other.energy_used;
    }
};

class Pathfinder {
private:
    Graph &graph;
    unordered_set<string> rest_points;
    unordered_set<string> charging_stations;
    int start_hour;
    string start, target;

    // visited[(node, energy_left, time)] = min_energy_used
    map<tuple<string,int,int>, int> visited;

public:
    Pathfinder(Graph &g,
               const string &s, const string &t,
               const unordered_set<string> &rest,
               const unordered_set<string> &charging,
               int hour)
        : graph(g), start(s), target(t),
          rest_points(rest), charging_stations(charging), start_hour(hour) {}

    void solve() {
        int start_time = start_hour * 60;

        priority_queue<State, vector<State>, greater<State>> pq;
        pq.push({0, start_time, start, MAX_ENERGY, {start}});

        while (!pq.empty()) {
            auto cur = pq.top(); pq.pop();

            int energy_used = cur.energy_used;
            int time = cur.time;
            string node = cur.node;
            int energy_left = cur.energy_left;
            vector<string> path = cur.path;

            // Goal check
            if (node == target) {
                print_result(energy_used, path);
                return;
            }

            auto state = make_tuple(node, energy_left, time);
            if (visited.count(state) && visited[state] <= energy_used) {
                continue;
            }
            visited[state] = energy_used;

            // Charging
            if (charging_stations.count(node)) {
                pq.push({energy_used, time, node, MAX_ENERGY, path});
            }

            // Rest
            if (rest_points.count(node)) {
                int current_hour = time / 60;
                int new_time = time;
                if (current_hour % 2 == 1) {
                    int minutes_to_next_hour = 60 - (time % 60);
                    new_time = time + minutes_to_next_hour;
                }
                pq.push({energy_used, new_time, node, energy_left, path});
            }

            // Explore neighbors
            for (auto &edge : graph.adj[node]) {
                int base_cost = edge.w + edge.o;
                int hour = time / 60;
                int cost;
                if (hour % 2 == 1) cost = int(base_cost * 1.3);
                else cost = int(base_cost * 0.8);

                if (cost <= energy_left) {
                    auto new_path = path;
                    new_path.push_back(edge.to);
                    pq.push({energy_used + cost, time + 2, edge.to,
                             energy_left - cost, new_path});
                }
            }
        }

        cout << "Robot gagal dalam mencapai tujuan :(\n";
    }

private:
    void print_result(int energy_used, const vector<string> &path) {
        cout << "Total energi minimum: " << energy_used << "\n";
        cout << "Jalur: ";
        for (size_t i = 0; i < path.size(); i++) {
            if (i) cout << " -> ";
            cout << path[i];
        }
        cout << "\nWaktu tiba:\n";
        for (size_t i = 0; i < path.size(); i++) {
            cout << path[i] << " (menit " << i*2 << ")\n";
        }
    }
};

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int n, m;
    cin >> n >> m;
    Graph graph(n);

    for (int i = 0; i < m; i++) {
        string u, v;
        int w, o;
        cin >> u >> v >> w >> o;
        graph.add_edge(u, v, w, o);
    }

    string S, T;
    cin >> S >> T;

    // Rest points
    unordered_set<string> rest_points;
    {
        string line;
        getline(cin, line); // buang newline sisa
        getline(cin, line);
        stringstream ss(line);
        string x;
        while (ss >> x) {
            if (x != "-") rest_points.insert(x);
        }
    }

    // Charging stations
    unordered_set<string> charging_stations;
    {
        string line;
        getline(cin, line);
        stringstream ss(line);
        string x;
        while (ss >> x) {
            if (x != "-") charging_stations.insert(x);
        }
    }

    string mechanic, electrical;
    getline(cin, mechanic);
    getline(cin, electrical);

    int start_hour;
    cin >> start_hour;

    Pathfinder solver(graph, S, T, rest_points, charging_stations, start_hour);
    solver.solve();

    return 0;
}
