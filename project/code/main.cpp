#include "rrt.h"
#include <fstream>
#include <iostream>


void exportPathToCSV(std::vector<Node> path, std::string fileName){
    std::ofstream file(fileName);
    if (!file.is_open()){
        std::cerr << "Failed to open file for writing: " << fileName << std::endl;
        return;
    }

    file << "x,y,z\n";
    for (auto waypoint : path){
        file << waypoint.x << "," << waypoint.y << "," << waypoint.z << "\n";
    }

    file.close();
}

void exportStatePathToCSV(std::vector<stateNode> path, std::string fileName){
    std::ofstream file(fileName);
    if (!file.is_open()){
        std::cerr << "Failed to open file for writing: " << fileName << std::endl;
        return;
    }

    file << "x,v_x,y,v_y,z,v_z\n";
    for (auto waypoint : path){
        file << waypoint.state[0] << "," << waypoint.state[1] << "," << waypoint.state[2] << "," << waypoint.state[3] << "," << waypoint.state[4] << "," << waypoint.state[5] << "\n";
    }

    file.close();
}

void exportObstaclesToCSV(std::vector<std::pair<int, Obstacle3D>> obstacles, std::string fileName){
    std::ofstream file(fileName);
    if (!file.is_open()){
        std::cerr << "Failed to open file for writing: " << fileName << std::endl;
        return;
    }

    file << "id,face,x,y,z\n";
    for (auto obstacle : obstacles){
        int id = obstacle.first;
        for (int i = 0; i < obstacle.second.faces.size(); i++){
            for (auto& vertex : obstacle.second.faces[i]){
                file << id << "," << i << "," << vertex.x() << "," << vertex.y() << "," << vertex.z() << "\n";
            }
        }
    }
    file.close();
    std::cout << "Obstacles written to " << fileName << std::endl;
}

int main(){
    // Define obstacles
    std::vector<std::pair<int, Obstacle3D>> obstacles;
    obstacles.emplace_back(1, std::vector<std::vector<Eigen::Vector3d>>{
        {
            Eigen::Vector3d(2.0, 2.0, 2.0),
            Eigen::Vector3d(4.0, 2.0, 2.0),
            Eigen::Vector3d(4.0, 4.0, 2.0),
            Eigen::Vector3d(2.0, 4.0, 2.0)
        },
        {
            Eigen::Vector3d(2.0, 2.0, 2.0),
            Eigen::Vector3d(4.0, 2.0, 2.0),
            Eigen::Vector3d(2.0, 2.0, 4.0)
        },
        {
            Eigen::Vector3d(2.0, 2.0, 2.0),
            Eigen::Vector3d(2.0, 4.0, 2.0),
            Eigen::Vector3d(2.0, 2.0, 4.0)
        },
        {
            Eigen::Vector3d(4.0, 2.0, 2.0),
            Eigen::Vector3d(4.0, 4.0, 2.0),
            Eigen::Vector3d(2.0, 2.0, 4.0)
        },
        {
            Eigen::Vector3d(4.0, 4.0, 2.0),
            Eigen::Vector3d(2.0, 4.0, 2.0),
            Eigen::Vector3d(2.0, 2.0, 4.0)
        }
    });
    obstacles.emplace_back(2, std::vector<std::vector<Eigen::Vector3d>>{
        {
            Eigen::Vector3d(8.0, 6.0, 6.0),
            Eigen::Vector3d(10.0, 6.0, 6.0),
            Eigen::Vector3d(8.0, 6.0, 8.0),
            Eigen::Vector3d(10.0, 6.0, 8.0)
        },
        {
            Eigen::Vector3d(8.0, 6.0, 6.0),
            Eigen::Vector3d(10.0, 6.0, 6.0),
            Eigen::Vector3d(8.0, 8.0, 6.0),
            Eigen::Vector3d(10.0, 8.0, 6.0)
        },
        {
            Eigen::Vector3d(8.0, 8.0, 6.0),
            Eigen::Vector3d(10.0, 8.0, 6.0),
            Eigen::Vector3d(8.0, 8.0, 8.0),
            Eigen::Vector3d(10.0, 8.0, 8.0)
        },
        {
            Eigen::Vector3d(8.0, 6.0, 8.0),
            Eigen::Vector3d(10.0, 6.0, 8.0),
            Eigen::Vector3d(8.0, 8.0, 8.0),
            Eigen::Vector3d(10.0, 8.0, 8.0)
        },
        {
            Eigen::Vector3d(8.0, 6.0, 6.0),
            Eigen::Vector3d(8.0, 6.0, 8.0),
            Eigen::Vector3d(8.0, 8.0, 8.0),
            Eigen::Vector3d(8.0, 8.0, 6.0)
        },
        {
            Eigen::Vector3d(10.0, 6.0, 6.0),
            Eigen::Vector3d(10.0, 6.0, 8.0),
            Eigen::Vector3d(10.0, 8.0, 8.0),
            Eigen::Vector3d(10.0, 8.0, 6.0)
        }
    });

    // Define bounds
    double xMin = 0.0, xMax = 15.0;
    double yMin = 0.0, yMax = 15.0;
    double zMin = 0.0, zMax = 15.0;

    Eigen::Vector3d qInit(0.0, 0.0, 0.0);  // Start position
    Eigen::Vector3d qGoal(10.0, 10.0, 10.0);  // Goal position
    // Create the 3D motion planning problem
    Problem3D problem(qInit, qGoal, obstacles, xMin, xMax, yMin, yMax, zMin, zMax);

    int n = 100000;
    // step size
    double r = 1.0;
    double epsilon = 0.5; // Goal Radius
    double p = 0.5; // Goal Bias
    double delta = 0.08;
    double gamma = 0.8;
    double kdlat = 0.02;
    double kdvert = 0.04;

    std::vector<stateNode> empty;

    aStar astar(false, empty, 0.0);

    kinoAgent agent;
    agent.m = 2;
    agent.g = 9.8;
    agent.stateUpperBounds.resize(6);
    agent.stateUpperBounds << problem.xMax, 20.0, problem.yMax, 20.0, problem.zMax, 20.0;
    agent.stateLowerBounds.resize(6);
    agent.stateLowerBounds << problem.xMin, -20.0, problem.yMin, -20.0, problem.zMin, -20.0;
    agent.controlUpperBounds.resize(4);
    agent.controlUpperBounds << 50.0, 50.0, 50.0, 50.0;
    agent.controlLowerBounds.resize(4);
    agent.controlLowerBounds << -5.0, -5.0, -5.0, -5.0;
    agent.states.resize(6);
    agent.controls.resize(4);

    agent.A.resize(6,6);
    agent.A << 0, 1, 0, 0, 0, 0,
               0, -kdlat/agent.m, 0, 0, 0, 0,
               0, 0, 0, 1, 0, 0,
               0, 0, 0, -kdlat/agent.m, 0, 0,
               0, 0, 0, 0, 0, 1,
               0, 0, 0, 0, 0, -kdvert/agent.m;

    double perturbation = 1e-4;
    agent.A -= perturbation * Eigen::MatrixXd::Ones(agent.A.rows(), agent.A.cols());


    agent.B.resize(6,4);
    agent.B << 0, 0, 0, 0,
               -delta/agent.m, 0, delta/agent.m, 0,
               0, 0, 0, 0,
               0, -delta/agent.m, 0, delta/agent.m,
               0, 0, 0, 0,
               gamma/agent.m, gamma/agent.m, gamma/agent.m, gamma/agent.m;

    agent.K.resize(4,6);
    agent.K << 
        -253.818127017307, -135.982851403804,  75.5463354518414,  6.55238277044001,  156.681892902577,  31.7194150725946,
         76.2129906320235,   6.93764910358262, -757.949216821157, -202.517468455165, -130.947832053823, -12.4359696811349,
        243.027130636985,  134.041384067563, -58.9168918592542, -4.30497502052463, -104.045417995582,  -14.5513715652,
        -87.0039870123465,  -8.87911643982333, 774.578660413745,  204.76487620508,   183.584306960818,  29.6040131885295;

    agent.F.resize(4,3);
    agent.F <<
        -253.829049982562,  75.5354124865866,  156.670969937321,
         76.1910338830834, -757.971173570095, -130.969788802759,
           243.0399291633, -58.9040933329389, -104.032619469266,
        -86.9801547023459,  774.602492723744,  183.608139270815;


    astar = kinoRRT(problem, agent, n, epsilon, p);
    // astar = rrt(problem, n, epsilon, p);

    // if (astar.success){
        std::cout << "Path found!" << std::endl;
        std::cout << "Path length: " << astar.pathCost << std::endl;

        // exportPathToCSV(astar.path, "path.csv");
        exportStatePathToCSV(astar.statePath, "pathPreSmooth.csv");
        smoothStatePath(astar, problem, agent);
        exportStatePathToCSV(astar.statePath, "pathPostSmooth.csv");
        exportObstaclesToCSV(problem.obstacles, "obstacles.csv");
    // } else{
    //     std::cout << "No path found." << std::endl;
    // }

    return 0;
}