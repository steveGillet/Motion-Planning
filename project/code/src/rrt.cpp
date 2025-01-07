#include "rrt.h"
#include <cmath>
#include <random>
#include <iostream>
#include <map>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

bool isLineInFace(std::vector<Eigen::Vector3d> faceVertices, std::vector<Eigen::Vector3d> lineVertices){
    Eigen::Vector3d lineDirection = lineVertices[1] - lineVertices[0];
    for (int i = 0; i < faceVertices.size(); i++) {
        Eigen::Vector3d A = faceVertices[i];
        Eigen::Vector3d B = faceVertices[(i+1)%faceVertices.size()];
        Eigen::Vector3d C = faceVertices[(i+2)%faceVertices.size()];

        Eigen::Vector3d normal = (B-A).cross(C-A).normalized();

        double denom = normal.dot(lineDirection);
        double t = -1.0;
        if (std::abs(denom) > 1e-8){
            t = -normal.dot(lineVertices[0] - A) / denom;
        }

        if (t >= 0 && t <= 1){
            Eigen::Vector3d intersectionPoint = lineVertices[0] + (lineVertices[1] - lineVertices[0]) * t;
            Eigen::Vector3d v0 = C - A;
            Eigen::Vector3d v1 = B - A;
            Eigen::Vector3d v2 = intersectionPoint - A;

            double dot00 = v0.dot(v0);
            double dot01 = v0.dot(v1);
            double dot02 = v0.dot(v2);
            double dot11 = v1.dot(v1);
            double dot12 = v1.dot(v2);

            double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            if ((u >= 0) && (v >= 0) && (u + v < 1)) return true;
        }
    }
    return false;
}

double heuristic(Node node, Eigen::Vector3d goal){
    return std::sqrt(
        std::pow(goal.x() - node.x, 2) +
        std::pow(goal.y() - node.y, 2) +
        std::pow(goal.z() - node.z, 2) 
    );
}

double heuristicState(stateNode node, Eigen::Vector3d goal){
    return std::sqrt(
        std::pow(goal.x() - node.state[0], 2) +
        std::pow(goal.y() - node.state[1], 2) +
        std::pow(goal.z() - node.state[2], 2) 
    );
}

void aStar::search(Problem3D problem, std::vector<Node> nodes){
    std::vector<Node> openList;
    std::vector<Node> closedList;
    std::vector<double> gCost(nodes.size(), __DBL_MAX__);
    std::vector<double> fCost(nodes.size(), __DBL_MAX__);
    std::vector<int> backPointer(nodes.size(), -1);
    int startId = nodes[0].id;
    int goalId = nodes[nodes.size() - 1].id;

    Node& startNode = nodes[0];
    openList.push_back(startNode);
    gCost[startId] = 0.0;
    fCost[startId] = gCost[startId] + heuristic(startNode, problem.qGoal);
    backPointer[startId] = startId;

    while (!openList.empty()){
        Node current(-1, 0.0, 0.0, 0.0, {});
        double minCost = __DBL_MAX__;
        for(Node& item : openList){
            if(fCost[item.id] < minCost){
                minCost = fCost[item.id];
                current = item;
            }
        }
        
        auto it = std::find_if(openList.begin(), openList.end(), [&](Node& n) {
            return n.id == current.id;
        });
        openList.erase(it);
        closedList.push_back(current);
        
        if(current.id == goalId){
            this->success = true;
            this->pathCost = gCost[current.id];
            this->path.push_back(nodes[current.id]);

            int it = current.id;
            while(it != startId){
                this->path.push_back(nodes[backPointer[it]]);
                it = backPointer[it];
            }

            std::reverse(this->path.begin(), this->path.end());
            return;
        }

        for(int childIndex : current.children){
            Node& child = nodes[childIndex];
            bool childInClosedList = false;
            for(Node& compare : closedList){
                if(child.id == compare.id) childInClosedList = true;
            }
            if(!childInClosedList){
                bool childInOpenList = false;
                for(Node& compare : openList){
                    if(child.id == compare.id) childInOpenList = true;
                }

                double potentialG = gCost[current.id] + heuristic(current, Eigen::Vector3d(child.x, child.y, child.z));
                if(!childInOpenList){
                    openList.push_back(child);
                    backPointer[child.id] = current.id;
                    gCost[child.id] = potentialG;
                    fCost[child.id] = potentialG + heuristic(child, problem.qGoal); 
                } 
                else if (potentialG < gCost[child.id])
                {
                    backPointer[child.id] = current.id;
                    gCost[child.id] = potentialG;
                    fCost[child.id] = potentialG + heuristic(child, problem.qGoal);  
                }
            }
        }
    }
}

void aStar::searchState(Problem3D problem, std::vector<stateNode> nodes){
    std::vector<stateNode> openList;
    std::vector<stateNode> closedList;
    std::vector<double> gCost(nodes.size(), __DBL_MAX__);
    std::vector<double> fCost(nodes.size(), __DBL_MAX__);
    std::vector<int> backPointer(nodes.size(), -1);
    int startId = nodes[0].id;
    int goalId = nodes[nodes.size() - 1].id;

    stateNode& startNode = nodes[0];
    openList.push_back(startNode);
    gCost[startId] = 0.0;
    fCost[startId] = gCost[startId] + heuristicState(startNode, problem.qGoal);
    backPointer[startId] = startId;

    while (!openList.empty()){
        stateNode current(-1, {}, {});
        double minCost = __DBL_MAX__;
        for(stateNode& item : openList){
            if(fCost[item.id] < minCost){
                minCost = fCost[item.id];
                current = item;
            }
        }
        
        auto it = std::find_if(openList.begin(), openList.end(), [&](stateNode& n) {
            return n.id == current.id;
        });
        openList.erase(it);
        closedList.push_back(current);
        
        if(current.id == goalId){
            this->success = true;
            this->pathCost = gCost[current.id];
            this->statePath.push_back(nodes[current.id]);

            int it = current.id;
            while(it != startId){
                this->statePath.push_back(nodes[backPointer[it]]);
                it = backPointer[it];
            }

            std::reverse(this->statePath.begin(), this->statePath.end());
            return;
        }

        for(int childIndex : current.children){
            stateNode& child = nodes[childIndex];
            bool childInClosedList = false;
            for(stateNode& compare : closedList){
                if(child.id == compare.id) childInClosedList = true;
            }
            if(!childInClosedList){
                bool childInOpenList = false;
                for(stateNode& compare : openList){
                    if(child.id == compare.id) childInOpenList = true;
                }

                double potentialG = gCost[current.id] + heuristicState(current, Eigen::Vector3d(child.state[0], child.state[1], child.state[2]));
                if(!childInOpenList){
                    openList.push_back(child);
                    backPointer[child.id] = current.id;
                    gCost[child.id] = potentialG;
                    fCost[child.id] = potentialG + heuristicState(child, problem.qGoal); 
                } 
                else if (potentialG < gCost[child.id])
                {
                    backPointer[child.id] = current.id;
                    gCost[child.id] = potentialG;
                    fCost[child.id] = potentialG + heuristicState(child, problem.qGoal);  
                }
            }
        }
    }
}

aStar rrt(const Problem3D& problem, int n, double r, double epsilon, double p){
    Problem3D expandedProblem = problem;
    expandObstacles(expandedProblem, 0.1);

    static std::random_device rd;
    static std::mt19937 gen(rd());

    std::vector<Node> nodes;
    nodes.emplace_back(Node(0, problem.qInit.x(), problem.qInit.y(), problem.qInit.z(), {}));
    int currentNodeIndex = 1;

    int counter = 0;
    bool pathFound = false;

    while (counter < n && !pathFound) {
        Eigen::Vector3d point;
        std::bernoulli_distribution bernoulli(p);

        if (bernoulli(gen)) {
            point = problem.qGoal; // Biased towards the goal
        } else {
            std::uniform_real_distribution<> xdis(problem.xMin, problem.xMax);
            std::uniform_real_distribution<> ydis(problem.yMin, problem.yMax);
            std::uniform_real_distribution<> zdis(problem.zMin, problem.zMax);
            point = Eigen::Vector3d(xdis(gen), ydis(gen), zdis(gen));
        }

        double distance = __DBL_MAX__;
        int qNearIndex = 0;
        for(int i = 0; i < nodes.size(); i++){
            double currentDistance = (point - Eigen::Vector3d(nodes[i].x, nodes[i].y, nodes[i].z)).norm();
            if(distance > currentDistance){
                qNearIndex = i;
                distance = currentDistance;
            } 
        }

        Eigen::Vector3d qNear = Eigen::Vector3d(nodes[qNearIndex].x, nodes[qNearIndex].y, nodes[qNearIndex].z);
        Eigen::Vector3d step = r * (point - qNear).normalized();
        Eigen::Vector3d qNew = qNear + step;

        bool pointInObstacle = false;
        for (auto& obstacle : problem.obstacles) {
            for (auto& face : obstacle.second.faces){
                if (isLineInFace(face, {qNear, qNew})){
                    pointInObstacle = true;
                    break;
                } 
            }
        }

        if(!pointInObstacle){
            nodes.emplace_back(Node(currentNodeIndex, qNew.x(), qNew.y(), qNew.z(), {}));
            currentNodeIndex++;
            nodes[qNearIndex].children.push_back(currentNodeIndex);

            if ((qNew - problem.qGoal).norm() < epsilon) {
                pathFound = true;
                nodes.emplace_back(Node(currentNodeIndex, problem.qGoal.x(), problem.qGoal.y(), problem.qGoal.z(), {}));
            }

        }
        counter++;
    }

    std::vector<Node> empty;
    aStar astar(false, empty, 0.0);

    if(!pathFound){
        std::cout << "Path not found in rrt." << std::endl;
        return astar;
    }

    astar.search(problem, nodes);
    if(astar.success){
        smoothPath(astar, expandedProblem);
    }
    return astar;
}

Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d>& vertices) {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& vertex : vertices) {
        sum += vertex;
    }
    return sum / vertices.size();
}

std::vector<Eigen::Vector3d> expandFace(const std::vector<Eigen::Vector3d>& face, double margin) {
    Eigen::Vector3d centroid = computeCentroid(face);
    std::vector<Eigen::Vector3d> expandedFace;
    for (const auto& vertex : face) {
        Eigen::Vector3d direction = vertex - centroid;
        expandedFace.push_back(centroid + direction * (1.0 + margin));
    }
    return expandedFace;
}

void expandObstacles(Problem3D& problem, double margin) {
    for (auto& obstacle : problem.obstacles) {
        for (auto& face : obstacle.second.faces) {
            face = expandFace(face, margin);
        }
    }
}

void smoothPath(aStar& astar, Problem3D& problem) {
    const int maxIterations = 50;  // Number of smoothing attempts

    for (int j = 0; j < maxIterations; j++) {
        // Randomly pick two waypoints in the path
        if (astar.path.size() <= 2) {
            break;  // No smoothing needed for fewer than 3 waypoints
        }

        int index1 = std::rand() % astar.path.size();
        int index2 = std::rand() % astar.path.size();

        // Ensure index1 < index2
        if (index1 > index2) std::swap(index1, index2);
        if (index2 - index1 <= 1) continue;  // Skip consecutive points

        Eigen::Vector3d p1(astar.path[index1].x, astar.path[index1].y, astar.path[index1].z);
        Eigen::Vector3d p2(astar.path[index2].x, astar.path[index2].y, astar.path[index2].z);

        bool collisionFree = true;
        for (auto obstacle : problem.obstacles) {
            for (auto& face : obstacle.second.faces){
                if (isLineInFace(face, {p1, p2})){
                    collisionFree = false;
                    break;
                } 
            }
            if (!collisionFree) break;
        }

        // If the straight line is valid, remove the intermediate waypoints
        if (collisionFree) {
            astar.path.erase(astar.path.begin() + index1 + 1, astar.path.begin() + index2);
        }
    }
}

aStar kinoRRT(Problem3D& problem, kinoAgent& agent, int& n, double& epsilon, double& p) {
    int uSamples = 100;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    int numStates = agent.states.size();
    
    std::vector<stateNode> nodes;
    std::vector<Eigen::VectorXd> points;
    std::map<std::pair<int, int>, Eigen::VectorXd> storedControls;
    std::map<std::pair<int, int>, double> storedDurations;

    points.push_back(problem.qInit);
    Eigen::VectorXd initialState(6);
    initialState << problem.qInit.x(), 0.0, problem.qInit.y(), 0.0, problem.qInit.z(), 0.0;
    nodes.emplace_back(stateNode(0, initialState, {}));
    int currentNodeIndex = 1;

    // Sampling random points
    int counter = 0;
    bool pathFound = false;
    while (counter < n && !pathFound) {
        Eigen::VectorXd point(agent.states.size());
        std::bernoulli_distribution bernoulli(p);

        if(bernoulli(gen)){
            point = (Eigen::VectorXd(6) << problem.qGoal.x(), 0.0, problem.qGoal.y(), 0.0, problem.qGoal.z(), 0.0).finished();
        }
        else{
            for (int i = 0; i < agent.states.size(); i++) {
                double lower = agent.stateLowerBounds[i];
                double upper = agent.stateUpperBounds[i];
                std::uniform_real_distribution<> dis(lower, upper);
                point[i] = dis(gen);
            }
        }

        // std::cout << "point: " << point << std::endl;


        Eigen::VectorXd qNear = nodes[0].state;
        double closestDistance = __DBL_MAX__;
        int qNearIndex = 0;
        for(int i = 0; i < nodes.size(); i++){
            // double currentDistance = ((point.head<3>() - nodes[i].state.head<3>()).norm() +
            //               0.1 * (point.tail<3>() - nodes[i].state.tail<3>()).norm());  // Weights velocity lower
            double currentDistance = (point - nodes[i].state).norm();
            // double currentDistance = (Eigen::Vector3d(point[0], point[2], point[4]) - Eigen::Vector3d(nodes[i].state[0], nodes[i].state[2], nodes[i].state[4])).norm() +
            //                           0.1 * (Eigen::Vector3d(point[1], point[3], point[5]) - Eigen::Vector3d(nodes[i].state[1], nodes[i].state[3], nodes[i].state[5])).norm(); 
            if(currentDistance < closestDistance){
                qNear = nodes[i].state;
                qNearIndex = i;
                closestDistance = currentDistance;
            } 
        }

        // // // // // ANALYTICAL PSEUDO INVERSE B SOLUTION // // // //

        // Eigen::VectorXd potentialControl(4);
        // Eigen::VectorXd xDot = (point - qNear) / dt;
        // std::cout << "Sampled Point: " << point << std::endl << "Nearest Neighbor: " << qNear << std::endl << "DT: " << dt << std::endl; 
        // potentialControl = agent.B.completeOrthogonalDecomposition().pseudoInverse() * (xDot - agent.A * qNear); 
        // std::cout << "A*qNear: " << agent.A * qNear << std::endl;
        
        // std::cout << "control: " << potentialControl << std::endl;
        // Eigen::VectorXd xDotActual = agent.A * qNear + agent.B * potentialControl;
        // std::cout << "xDot: " << xDot << ", xDotActual: " << xDotActual << std::endl;
        // // potentialControl << agent.m*(point[3] - qNear[3])/dt, agent.m*(point[4] - qNear[4])/dt, agent.m*(point[5] - qNear[5])/dt + agent.m*agent.g;
        // bool controlInBounds = true;

        // for (int i = 0; i < agent.controlLowerBounds.size(); i++){
        //     if (potentialControl[i] < agent.controlLowerBounds[i]) controlInBounds = false;
        // }
        // for (int i = 0; i < agent.controlUpperBounds.size(); i++){
        //     if (potentialControl[i] > agent.controlUpperBounds[i]) controlInBounds = false;
        // }
        // // std::vector<double> potentialDts;

        // // // // // MINIMUM ENERGY SOLUTION // // // //

        std::uniform_real_distribution<> randomDT(1, 10);
        double dt = randomDT(gen);
        Eigen::MatrixXd G = solveLyapunovNumerically(agent.A, agent.B, dt);
        G += 1e-6 * Eigen::MatrixXd::Identity(G.rows(), G.cols()); 

        Eigen::VectorXd u = agent.B.transpose() * (agent.A.transpose() * dt).exp() * G.inverse() * ((-agent.A * dt).exp() * point - qNear);

        Eigen::VectorXd xDot = agent.A * qNear + agent.B * u;
        Eigen::VectorXd qNew = qNear + xDot;

        bool controlInvalid = false;
        for (int i = 0; i < qNew.size(); i++){
            if(qNew[i] < agent.stateLowerBounds[i]){
                controlInvalid = true;
            } 
            if(qNew[i] > agent.stateUpperBounds[i]){
                controlInvalid = true;
            } 
        }
        
        for (auto obstacle : problem.obstacles) {
            for (auto& face : obstacle.second.faces){
                if (isLineInFace(face, {{qNear[0], qNear[2], qNear[4]}, {qNew[0], qNew[2], qNew[4]}})){
                    controlInvalid = true;
                    break;
                } 
            }
            if (controlInvalid) break;
        }


        // // // // // // RANDOM SOLUTION // // // //

        // std::vector<Eigen::VectorXd> potentialControls;
        // std::vector<Eigen::VectorXd> potentialQnews;
        // std::vector<double> potentialDts;
        // int maxAttempts = 200; // will probably have to implement if an infinite loop happens
        // int controlCounter = 0;

        // while(potentialControls.size() < uSamples && controlCounter < maxAttempts){
        //     Eigen::VectorXd control(agent.controls.size());
        //     for (int i = 0; i < agent.controls.size(); i++) {
        //         double lower = agent.controlLowerBounds[i];
        //         double upper = agent.controlUpperBounds[i];
        //         std::uniform_real_distribution<> dis(lower, upper);
        //         control[i] = dis(gen);
        //     }

        //     std::uniform_real_distribution<> randomDT(.05, 2);
        //     double dt = randomDT(gen);

        //     Eigen::VectorXd potentialQnew = qNear;
        //     Eigen::VectorXd previousState = potentialQnew;

        //     potentialQnew = qNear + (agent.A * qNear + agent.B * control)*dt; 
        //     // std::cout << "control: " << control << std::endl;
        //     // std::cout << "potential q new: " << potentialQnew << std::endl;



        //     bool controlInvalid = false;
        //     for (int i = 0; i < potentialQnew.size(); i++){
        //         if(potentialQnew[i] < agent.stateLowerBounds[i]){
        //             controlInvalid = true;
        //         } 
        //         if(potentialQnew[i] > agent.stateUpperBounds[i]){
        //             controlInvalid = true;
        //         } 
        //     }
            
        //     for (auto obstacle : problem.obstacles) {
        //         for (auto& face : obstacle.second.faces){
        //             if (isLineInFace(face, {{qNear[0], qNear[1], qNear[2]}, {potentialQnew[0], potentialQnew[1], potentialQnew[2]}})){
        //                 controlInvalid = true;
        //                 break;
        //             } 
        //         }
        //         if (controlInvalid) break;
        //     }

        //     if(!controlInvalid){
        //         potentialControls.push_back(control);
        //         potentialQnews.push_back(potentialQnew);
        //         potentialDts.push_back(dt);
        //     }

        //     controlCounter++;
        // }

        // if(potentialControls.size() > 0){
            
            // double closestDistanceToQnew = __DBL_MAX__;
            // Eigen::VectorXd bestControl;
            // Eigen::VectorXd qNew;
            // double bestDt;
            // for (int i = 0; i < potentialControls.size(); i++) {
            //     // double distance = ((point.head<3>() - potentialQnews[i].head<3>()).norm() +
            //     //                     0.1 * (point.tail<3>() - potentialQnews[i].tail<3>()).norm());
            //     // double distance = (Eigen::Vector3d(point[0], point[2], point[4]) - Eigen::Vector3d(potentialQnews[i][0], potentialQnews[i][2], potentialQnews[i][4])).norm() +
            //     //                    0.1 * (Eigen::Vector3d(point[1], point[3], point[5]) - Eigen::Vector3d(potentialQnews[i][1], potentialQnews[i][3], potentialQnews[i][5])).norm(); 
            //     double distance = (point - potentialQnews[i]).norm();
            //     if (distance < closestDistanceToQnew) {
            //         closestDistanceToQnew = distance;
            //         bestControl = potentialControls[i];
            //         bestDt = potentialDts[i];
            //         qNew = potentialQnews[i];
            //     }
            // }

            // std::cout << "qNew: " << qNew << std::endl;
            // std::cout << "best control: " << bestControl << std::endl;

        if(!controlInvalid){
            nodes.emplace_back(stateNode(currentNodeIndex, qNew, {}));            
            nodes[qNearIndex].children.push_back(currentNodeIndex);

            storedControls[{qNearIndex, currentNodeIndex}] = u;
            storedDurations[{qNearIndex, currentNodeIndex}] = dt;
            if ((Eigen::Vector3d(qNew[0], qNew[2], qNew[4]) - problem.qGoal).norm() < epsilon) {
                std::cout << "Reached the goal" << std::endl;
                pathFound = true;
            }


            currentNodeIndex++;
        }
            

        counter++;
    }


    std::vector<stateNode> empty;
    aStar astar(false, empty, 0.0);

    if(!pathFound){
        std::cout << "Path not found in rrt." << std::endl;
        return astar;
    }

    astar.searchState(problem, nodes);
    
    return astar;
}

void smoothStatePath(aStar& astar, Problem3D& problem, kinoAgent& agent) {
    const int maxIterations = 100;  // Number of smoothing attempts
    static std::random_device rd;
    static std::mt19937 gen(rd());
    // bool smoothingFound = false;


    for (int j = 0; j < maxIterations; j++) {
        // Randomly pick two waypoints in the path

        if (astar.statePath.size() <= 2) {
            break;  // No smoothing needed for fewer than 3 waypoints
        }

        int index1 = std::rand() % astar.statePath.size();
        int index2 = std::rand() % astar.statePath.size();

        // Ensure index1 < index2
        if (index1 > index2) std::swap(index1, index2);
        if (index2 - index1 <= 1) continue;  // Skip consecutive points

        Eigen::VectorXd p1(6);
        p1 = astar.statePath[index1].state;
        Eigen::VectorXd p2(6);
        p2 = astar.statePath[index2].state;

        bool collisionFree = true;
        for (auto obstacle : problem.obstacles) {
            for (auto& face : obstacle.second.faces){
                if (isLineInFace(face, {Eigen::Vector3d(p1[0], p1[2], p1[4]), Eigen::Vector3d(p2[0], p2[2], p2[4])})){
                    collisionFree = false;
                    break;
                } 
            }
            if (!collisionFree) break;
        }

        std::uniform_real_distribution<> randomDT(1, 10);
        double dt = randomDT(gen);

        Eigen::MatrixXd G = solveLyapunovNumerically(agent.A, agent.B, dt);
        G += 1e-6 * Eigen::MatrixXd::Identity(G.rows(), G.cols()); 

        Eigen::VectorXd potentialControl = agent.B.transpose() * (agent.A.transpose() * dt).exp() * G.inverse() * ((-agent.A * dt).exp() * p2 - p1);

        // potentialControl << agent.m*(p2[3] - p1[3])/dt, agent.m*(p2[4] - p1[4])/dt, agent.m*(p2[5] - p1[5])/dt + agent.m*agent.g;
        bool controlInBounds = true;

        for (int i = 0; i < agent.controlLowerBounds.size(); i++){
            if (potentialControl[i] < agent.controlLowerBounds[i]) controlInBounds = false;
        }
        for (int i = 0; i < agent.controlUpperBounds.size(); i++){
            if (potentialControl[i] > agent.controlUpperBounds[i]) controlInBounds = false;
        }

        // If the straight line is valid, remove the intermediate waypoints
        if (collisionFree && controlInBounds) {
            astar.statePath.erase(astar.statePath.begin() + index1 + 1, astar.statePath.begin() + index2);
            // smoothingFound = true;
        }
    }
}


Eigen::MatrixXd solveLyapunovNumerically(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double t) {
    double t0 = 0;
    int N = 100;
    double dt = t / N;
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(A.rows(), A.cols());

    G += integrand(A, B, t0, t0) * dt / 2.0;
    
    for (int i = 0; i < N; i++) {
        double tau = t0 + i * dt;
        G += integrand(A, B, t0, tau) * dt;
    }

    G += integrand(A, B, t, t) * dt / 2.0;
    
    return G;
}

Eigen::MatrixXd integrand(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double t0, double tau) {
    return (A * (t0 - tau)).exp() * B * B.transpose() * (A.transpose() * (t0 - tau)).exp();
}
