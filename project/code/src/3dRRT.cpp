#include "3dRRT.h"
#include <cmath>
#include <iostream>
#include <random>


// ----------------------- Path3D Definitions -----------------------
double Path3D::getPathLength() const {
    double length = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        const Node& current = waypoints[i];
        const Node& previous = waypoints[i - 1];
        length += std::sqrt(
            std::pow(current.x - previous.x, 2) +
            std::pow(current.y - previous.y, 2) +
            std::pow(current.z - previous.z, 2)
        );
    }
    return length;
}

void Path3D::print() const {
    std::cout << "Path3D:" << std::endl;
    for (const auto& waypoint : waypoints) {
        std::cout << waypoint << std::endl;
    }
}

void Path3D::clear() {
    waypoints.clear();
}

// void Path3D::smoothPath(int iterations, Problem3D problem) {
//     if (waypoints.size() <= 2) {
//         return; // No smoothing needed for fewer than 3 waypoints
//     }

//     for (int iter = 0; iter < iterations; ++iter) {
//         for (size_t i = 1; i < waypoints.size() - 1; ++i) {
//             Node prev = waypoints[i - 1];
//             Node next = waypoints[i + 1];
//             if(prev.x )
//             waypoints[i].x = (1 - alpha) * waypoints[i].x + alpha * 0.5 * (prev.x + next.x);
//             waypoints[i].y = (1 - alpha) * waypoints[i].y + alpha * 0.5 * (prev.y + next.y);
//             waypoints[i].z = (1 - alpha) * waypoints[i].z + alpha * 0.5 * (prev.z + next.z);
//         }
//     }
// }

// ----------------------- Obstacle3D Definitions -----------------------
Obstacle3D::Obstacle3D(const std::vector<Eigen::Vector3d>& vertices_) : vertices(vertices_) {}

bool Obstacle3D::contains(const Eigen::Vector3d& point) const {
    Eigen::Vector3d min = vertices[0];
    Eigen::Vector3d max = vertices[0];

    for (const auto& vertex : vertices) {
        min = min.cwiseMin(vertex);
        max = max.cwiseMax(vertex);
    }

    return (point.x() >= min.x() && point.x() <= max.x() &&
            point.y() >= min.y() && point.y() <= max.y() &&
            point.z() >= min.z() && point.z() <= max.z());
}

void Obstacle3D::print() const {
    std::cout << "Obstacle3D vertices: ";
    for (const auto& vertex : vertices) {
        std::cout << "(" << vertex.x() << ", " << vertex.y() << ", " << vertex.z() << ") ";
    }
    std::cout << std::endl;
}

// ----------------------- Problem3D Definitions -----------------------
Problem3D::Problem3D(
    const Eigen::Vector3d& q_init_,
    const Eigen::Vector3d& q_goal_,
    const std::vector<std::pair<int, Obstacle3D>>& obstacles_,
    double x_min_, double x_max_,
    double y_min_, double y_max_,
    double z_min_, double z_max_
) : q_init(q_init_), q_goal(q_goal_), obstacles(obstacles_),
    x_min(x_min_), x_max(x_max_),
    y_min(y_min_), y_max(y_max_),
    z_min(z_min_), z_max(z_max_) {}

bool Problem3D::isValid(const Eigen::Vector3d& point) const {
    if (point.x() < x_min || point.x() > x_max ||
        point.y() < y_min || point.y() > y_max ||
        point.z() < z_min || point.z() > z_max) {
        return false; // Out of bounds
    }

    for (const auto& obstacle : obstacles) {
        if (obstacle.second.contains(point)) {
            return false; // Inside an obstacle
        }
    }

    return true; // Valid point
}

void Problem3D::print() const {
    std::cout << "Problem3D:" << std::endl;
    std::cout << "  q_init: (" << q_init.x() << ", " << q_init.y() << ", " << q_init.z() << ")" << std::endl;
    std::cout << "  q_goal: (" << q_goal.x() << ", " << q_goal.y() << ", " << q_goal.z() << ")" << std::endl;
    std::cout << "  x bounds: [" << x_min << ", " << x_max << "]" << std::endl;
    std::cout << "  y bounds: [" << y_min << ", " << y_max << "]" << std::endl;
    std::cout << "  z bounds: [" << z_min << ", " << z_max << "]" << std::endl;
    std::cout << "  Obstacles:" << std::endl;
    for (const auto& obstacle : obstacles) {
        obstacle.second.print();
    }
}

// ----------------------- CustomHeuristic Definitions -----------------------
CustomHeuristic::CustomHeuristic(const std::unordered_map<Node, double, NodeHash>& values) : heuristic_values(values) {}

double CustomHeuristic::operator()(Node node) const {
    auto it = heuristic_values.find(node);
    if (it != heuristic_values.end()) {
        return it->second;
    }
    return 0.0; // Default heuristic value
}

// ----------------------- MyAStarAlgo Definitions -----------------------
GraphSearchResult MyAStarAlgo::search(const Problem3D& problem, const CustomHeuristic& heuristic) {
    GraphSearchResult result = {false, {}, 0.0};

    std::priority_queue<NodeCost> open_list;
    std::unordered_map<int, double> g_cost;
    std::unordered_map<int, int> came_from; // Use int keys

    int start_id = 0;
    int goal_id = 1;

    open_list.push(NodeCost(0.0, Node(start_id, problem.q_init.x(), problem.q_init.y(), problem.q_init.z())));
    g_cost[start_id] = 0.0;

    while (!open_list.empty()) {
        NodeCost current = open_list.top();
        open_list.pop();

        int current_id = current.node.id;

        if (current_id == goal_id) {
            result.success = true;
            result.path_cost = g_cost[current_id];

            int trace = current_id;
            while (trace != start_id) {
                result.node_path.push_back(Node(trace, problem.graph->adjacency_list[trace][0], 0, 0)); // Example, adjust as needed
                trace = came_from[trace];
            }
            result.node_path.push_back(Node(start_id, problem.q_init.x(), problem.q_init.y(), problem.q_init.z()));
            std::reverse(result.node_path.begin(), result.node_path.end());
            return result;
        }

        for (auto& neighbor : problem.graph->children(current_id)) {
            double tentative_g = g_cost[current_id] + problem.graph->cost(current_id, neighbor);

            if (!g_cost.count(neighbor) || tentative_g < g_cost[neighbor]) {
                g_cost[neighbor] = tentative_g;
                double f_cost = tentative_g + heuristic(Node(neighbor, 0, 0, 0)); // Adjust heuristic as needed
                open_list.push(NodeCost(f_cost, Node(neighbor, 0, 0, 0))); // Adjust as needed
                came_from[neighbor] = current_id;
            }
        }
    }

    return result; // If no path is found
}


// ----------------------- RRT Definitions -----------------------
std::tuple<Path3D, GraphSearchResult, std::unordered_map<int, Eigen::Vector3d, NodeHash>>
MyRRT::plan(const Problem3D& problem, int n, double r, double epsilon, double p) {
    Path3D path;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    std::unordered_map<int, Eigen::Vector3d, NodeHash> nodes; // Nodes indexed by IDs
    std::vector<std::tuple<int, int, double>> edges; // Edges as (from, to, cost)
    problem.graph->addNode(0);
    nodes[0] = problem.q_init;
    int currentNodeIndex = 1;

    int counter = 0;
    bool pathFound = false;

    while (counter < n && !pathFound) {
        Eigen::Vector3d point;
        std::bernoulli_distribution bernoulli(p);

        if (bernoulli(gen)) {
            point = problem.q_goal; // Biased towards the goal
        } else {
            std::uniform_real_distribution<> xdis(problem.x_min, problem.x_max);
            std::uniform_real_distribution<> ydis(problem.y_min, problem.y_max);
            std::uniform_real_distribution<> zdis(problem.z_min, problem.z_max);
            point = Eigen::Vector3d(xdis(gen), ydis(gen), zdis(gen));
        }

        // Find the nearest node
        int nearestIndex = 0;
        double minDistance = std::numeric_limits<double>::max();
        for (const auto& [index, node] : nodes) {
            double distance = (point - node).norm();
            if (distance < minDistance) {
                nearestIndex = index;
                minDistance = distance;
            }
        }

        Eigen::Vector3d qNear = nodes[nearestIndex];
        Eigen::Vector3d direction = (point - qNear).normalized();
        Eigen::Vector3d qNew = qNear + r * direction;

        if (problem.isValid(qNew)) {
            nodes[currentNodeIndex] = qNew;
            edges.push_back({nearestIndex, currentNodeIndex, r});
            problem.graph->addNode(currentNodeIndex);
            problem.graph->addEdge(nearestIndex, currentNodeIndex, r);
            currentNodeIndex++;

            if ((qNew - problem.q_goal).norm() < epsilon) {
                pathFound = true;
                nodes[currentNodeIndex] = problem.q_goal;
                edges.push_back({currentNodeIndex - 1, currentNodeIndex, (qNew - problem.q_goal).norm()});
            }
        }

        counter++;
    }

    if (!pathFound) {
        return std::make_tuple(path, GraphSearchResult{}, nodes); // Return empty path if no path is found
    }

    // Backtrack to construct the path
    int trace = currentNodeIndex;
    while (trace != 0) {
        path.waypoints.push_back(Node(trace, nodes[trace].x(), nodes[trace].y(), nodes[trace].z()));
        for (const auto& [from, to, cost] : edges) {
            if (to == trace) {
                trace = from;
                break;
            }
        }
    }

    std::reverse(path.waypoints.begin(), path.waypoints.end());
    std::unordered_map<Node, double, NodeHash> heuristic_values = {
        {Node(0, problem.q_init.x(), problem.q_init.y(), problem.q_init.z()), (problem.q_goal - problem.q_init).norm()},
        {Node(1, problem.q_goal.x(), problem.q_goal.y(), problem.q_goal.z()), 0.0} // Goal has 0 heuristic
    };
    MyAStarAlgo aStar;
    GraphSearchResult result = aStar.search(problem, CustomHeuristic(heuristic_values));
    return std::make_tuple(path, result, nodes);
}