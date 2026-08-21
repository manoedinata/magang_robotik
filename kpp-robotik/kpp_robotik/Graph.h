#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <vector>
#include <map>
#include <tuple>

class Graph {
    private:
        // Adjacency list: maps a node to a list of its neighbors.
        // {destination_node, weight, obstacle_cost}.
        std::map<
            std::string,
            std::vector<
                std::tuple<
                    std::string,
                    int,
                    int
                >
            >
        > adj;

    public:
        Graph() = default; // Default constructor

        // Adds an edge between two nodes
        void add_edge(
            const std::string& u,
            const std::string& v,
            int w,
            int o
        );

        // Gets all neighbors of a given node
        // returns: [(neighbor_node, w, o), ...]
        const std::vector<
                std::tuple<
                    std::string,
                    int,
                    int
                >
            >& get_neighbors(const std::string& node) const;
    };

#endif // GRAPH_H
