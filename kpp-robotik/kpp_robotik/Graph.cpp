#include "Graph.h"

// Adds an edge to the adjacency list
void Graph::add_edge(
    const std::string& u,
    const std::string& v,
    int w,
    int o
){
    adj[u].emplace_back(v, w, o);
}

// Retrieves neighbors for a node. Returns an empty vector if the node has no neighbors.
const std::vector<
    std::tuple<
        std::string,
        int,
        int
    >
>& Graph::get_neighbors(const std::string& node) const
{
    static const std::vector<std::tuple<std::string, int, int>> empty_vector;
    auto it = adj.find(node);
    if (it != adj.end()) {
        return it->second;
    } else {
        return empty_vector;
    }
}
