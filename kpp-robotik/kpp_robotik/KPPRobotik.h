#ifndef KPP_ROBOTIK_H
#define KPP_ROBOTIK_H

#include "Graph.h"
#include <string>
#include <vector>
#include <set>

// Struct: State node saat ini (current node)
struct State {
    int energy_used;
    int time;
    std::string node;
    int energy_left;
    std::vector<std::string> path;
    std::vector<int> timeline;

    // Overload operator untuk menjadikan Priority Queue menjadi min-heap
    // (nilai terkecil punya prioritas lebih banyak)
    // Ref: https://cboard.cprogramming.com/cplusplus-programming/111585-overloading-priority-queue.html
    bool operator>(const State& other) const {
        return energy_used > other.energy_used;
    }
};

// Class object: Main KPPRobotik
class KPPRobotik {
    private:
        Graph graph;
        static const int MAX_ENERGY = 1000;
        static const int SPEED = 100; // 100 m / menit (CONTOH)

    public:
        KPPRobotik() = default;

        // Method to add an edge to the internal graph object
        void add_edge(
            const std::string& u,
            const std::string& v,
            int w,
            int o
        );

        // The main algorithm to find the best route
        void get_best_routes(
            const std::string& start,
            const std::string& target,
            const std::set<std::string>& rest_points,
            const std::set<std::string>& charging_stations,
            int start_hour
        );
};

#endif // KPP_ROBOTIK_H
