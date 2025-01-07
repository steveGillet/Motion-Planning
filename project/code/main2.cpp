#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <map>
#include "3dRRT.h"
#include <fstream>

// Function to export path to a CSV
void exportPathToCSV(const Path3D& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    file << "x,y,z\n";  // Header
    for (const auto& waypoint : path.waypoints) {
        file << waypoint.x << "," << waypoint.y << "," << waypoint.z << "\n";
    }

    file.close();
}

// Write vertices to a CSV file
void exportObstaclesToCSV(const std::string& filename, const std::vector<std::pair<int, Obstacle3D>>& obstacles) {
    std::ofstream file(filename);
    file << "id,x,y,z\n";
    for (const auto& obstacle : obstacles) {
        int id = obstacle.first;
        for (const auto& vertex : obstacle.second.vertices) {
            file << id << "," << vertex.x() << "," << vertex.y() << "," << vertex.z() << "\n";
        }
    }
    file.close();
    std::cout << "Obstacles written to " << filename << std::endl;
}

int main() {

    // Define obstacles
    std::vector<std::pair<int, Obstacle3D>> obstacles;
    obstacles.emplace_back(1, std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(2.0, 2.0, 2.0),
        Eigen::Vector3d(4.0, 2.0, 2.0),
        Eigen::Vector3d(4.0, 4.0, 2.0),
        Eigen::Vector3d(2.0, 4.0, 2.0),
        Eigen::Vector3d(2.0, 2.0, 4.0)
    });
    obstacles.emplace_back(2, std::vector<Eigen::Vector3d>{
        Eigen::Vector3d(8.0, 8.0, 8.0),
        Eigen::Vector3d(8.0, 8.0, 6.0),
        Eigen::Vector3d(8.0, 6.0, 8.0),
        Eigen::Vector3d(8.0, 6.0, 6.0),
        Eigen::Vector3d(10.0, 8.0, 8.0),
        Eigen::Vector3d(10.0, 8.0, 6.0),
        Eigen::Vector3d(10.0, 6.0, 8.0),
        Eigen::Vector3d(10.0, 6.0, 6.0)
    });

    // Define bounds
    double x_min = 0.0, x_max = 15.0;
    double y_min = 0.0, y_max = 15.0;
    double z_min = 0.0, z_max = 15.0;

    Eigen::Vector3d q_init(0.0, 0.0, 0.0);  // Start position
    Eigen::Vector3d q_goal(10.0, 10.0, 10.0);  // Goal position
    // Create the 3D motion planning problem
    Problem3D problem(q_init, q_goal, obstacles, x_min, x_max, y_min, y_max, z_min, z_max);

    // Define start and goal positions

    problem.graph = std::make_shared<Graph>();
    
    // Print the problem setup
    problem.print();

    // RRT parameters
    int n_samples = 1000;   // Number of random samples
    double step_size = 1.0; // Step size for RRT
    double epsilon = 0.5;   // Goal proximity
    double goal_bias = 0.1; // Probability of sampling the goal

    // Run RRT
    std::cout << "Running RRT..." << std::endl;
    Path3D path;
    GraphSearchResult result;
    MyRRT rrt;
    std::unordered_map<int, Eigen::Vector3d, NodeHash> nodes;
    std::tie(path, result, nodes) = rrt.plan(problem, n_samples, step_size, epsilon, goal_bias);

    // Print and smooth the path
    if (result.success) {
        std::cout << "Path found!" << std::endl;
        path.print();
        std::cout << "Path length: " << path.getPathLength() << std::endl;

        // std::cout << "Smoothing path..." << std::endl;
        // path.smoothPath();
        // std::cout << "Smoothed path length: " << path.getPathLength() << std::endl;
    } else {
        std::cout << "No path found." << std::endl;
    }
    exportPathToCSV(path, "path.csv");
    exportObstaclesToCSV("obstacles.csv", problem.obstacles);

    return 0;
}
