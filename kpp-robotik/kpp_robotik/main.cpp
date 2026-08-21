#include <iostream>
#include <string>
#include <sstream>
#include <set>

#include "KPPRobotik.h"

// Helper function to read a line of space-separated strings into a set
void read_points(std::set<std::string>& points_set) {
    std::string line, point;
    std::getline(std::cin, line);

    if (line == "-") return;

    std::stringstream ss(line);
    while (ss >> point) {
        points_set.insert(point);
    }
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    // Initialize object
    KPPRobotik kpp_robot;

    // N, M
    int n, m;
    std::cin >> n >> m;

    // M baris, berisi graph node & edge
    for (int i = 0; i < m; i++) {
        std::string u, v;
        int w, o;
        std::cin >> u >> v >> w >> o;
        kpp_robot.add_edge(u, v, w, o);
    }

    // S, T (Start, Target)
    std::string start_node, target_node;
    std::cin >> start_node >> target_node;
    std::cin.ignore();

    // Rest points
    std::set<std::string> rest_points;
    read_points(rest_points);

    // Charging stations
    std::set<std::string> charging_stations;
    read_points(charging_stations);

    // Mechanic & Electrical (UNUSED)
    std::string mechanic;
    std::string electrical;
    std::getline(std::cin, mechanic);
    std::getline(std::cin, electrical);

    // Start hour
    int start_hour;
    std::cin >> start_hour;

    // Letsgoo dawgg
    kpp_robot.get_best_routes(
        start_node,
        target_node,
        rest_points,
        charging_stations,
        start_hour
    );

    return 0;
}
