#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include <vector>
#include <unordered_map>
#include <queue>
#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include <memory>
#include <functional>
#include <utility>

namespace std {
    template <>
    struct hash<std::pair<int, int>> {
        size_t operator()(const std::pair<int, int>& p) const {
            auto h1 = std::hash<int>()(p.first);
            auto h2 = std::hash<int>()(p.second);
            return h1 ^ (h2 << 1); // Combine the two hashes
        }
    };
}

// Node struct definition
struct Node {
    int id;
    double x;
    double y;
    double z;

    Node() : id(0), x(0.0), y(0.0), z(0.0) {}

    Node(int id_, double x_ = 0.0, double y_ = 0.0, double z_ = 0.0)
        : id(id_), x(x_), y(y_), z(z_) {}

    bool operator==(const Node& other) const {
        return id == other.id;
    }

    bool operator!=(const Node& other) const {
        return !(*this == other);
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
        os << "Node(ID: " << node.id << ", x: " << node.x << ", y: " << node.y;
        if (node.z != 0.0) os << ", z: " << node.z;
        os << ")";
        return os;
    }
};

struct NodeHash {
    std::size_t operator()(const Node& node) const {
        return std::hash<int>()(node.id);
    }
};


class Graph {
public:
    std::unordered_map<int, std::vector<int>> adjacency_list;
    std::unordered_map<std::pair<int, int>, double> edge_costs;

    Graph() = default;

    void addNode(int id) {
        if (adjacency_list.find(id) == adjacency_list.end()) {
            adjacency_list[id] = {};
        }
    }

    void addEdge(int from, int to, double cost) {
        adjacency_list[from].push_back(to);
        edge_costs[{from, to}] = cost;
    }

    const std::vector<int>& children(int node_id) const {
        return adjacency_list.at(node_id);
    }

    double cost(int from, int to) const {
        return edge_costs.at({from, to});
    }
};


// Path3D struct definition
struct Path3D {
    std::vector<Node> waypoints;

    double getPathLength() const;
    void print() const;
    void clear();
    // void smoothPath(int iterations = 10, double alpha = 0.1);
};

// Obstacle3D struct definition
struct Obstacle3D {
    std::vector<Eigen::Vector3d> vertices;

    Obstacle3D(const std::vector<Eigen::Vector3d>& vertices_);
    bool contains(const Eigen::Vector3d& point) const;
    void print() const;
};

// Problem3D struct definition
struct Problem3D {
    Eigen::Vector3d q_init;
    Eigen::Vector3d q_goal;
    std::shared_ptr<Graph> graph;
    std::vector<std::pair<int, Obstacle3D>> obstacles;
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;

    Problem3D(
        const Eigen::Vector3d& q_init_,
        const Eigen::Vector3d& q_goal_,
        
        const std::vector<std::pair<int, Obstacle3D>>& obstacles_,
        double x_min_, double x_max_,
        double y_min_, double y_max_,
        double z_min_, double z_max_
    );

    bool isValid(const Eigen::Vector3d& point) const;
    void print() const;
};

struct NodeCost {
    double cost;
    Node node;

    NodeCost(double cost_, Node node_) : cost(cost_), node(std::move(node_)) {}

    // Add comparison operator for priority queue
    bool operator<(const NodeCost& other) const {
        return cost > other.cost; // Lower cost has higher priority
    }
};

// GraphSearchResult struct for search results
struct GraphSearchResult {
    bool success;
    std::vector<Node> node_path;
    double path_cost;
};

// Custom heuristic class for A*
class CustomHeuristic {
private:
    std::unordered_map<Node, double, NodeHash> heuristic_values;

public:
    CustomHeuristic(const std::unordered_map<Node, double, NodeHash>& values);
    double operator()(Node node) const;
};

// A* algorithm class
class MyAStarAlgo {
public:
    GraphSearchResult search(const Problem3D& problem, const CustomHeuristic& heuristic);
};

// RRT planner class
class MyRRT {
public:
    std::tuple<Path3D, GraphSearchResult, std::unordered_map<int, Eigen::Vector3d, NodeHash>> plan(
        const Problem3D& problem,
        int n, double r, double epsilon, double p
    );
};

// Function declarations
// double heuristic(const Node& node, const Eigen::Vector3d& goal);
// void smoothPath(Path3D& path, const Problem3D& problem);

#endif // PATHPLANNING_H
