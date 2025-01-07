#include <Eigen/Dense>
#include <vector>

struct Node {
    int id;
    double x;
    double y;
    double z;
    std::vector<int> children;

    Node(int idIn, double xIn, double yIn, double zIn, std::vector<int> childrenIn){
        id = idIn;
        x = xIn;
        y = yIn;
        z = zIn;
        children = childrenIn;
    }
};

struct stateNode {
    int id;
    Eigen::VectorXd state;
    std::vector<int> children;

    stateNode(int idIn, Eigen::VectorXd stateIn, std::vector<int> childrenIn){
        id = idIn;
        state = stateIn;
        children = childrenIn;
    }
};

struct Obstacle3D{
    std::vector<std::vector<Eigen::Vector3d>> faces;

    Obstacle3D(std::vector<std::vector<Eigen::Vector3d>> facesIn){
        faces = facesIn;
    }
};

bool isLineInFace(std::vector<Eigen::Vector3d> faceVertices, std::vector<Eigen::Vector3d> lineVertices);

struct Problem3D{
    Eigen::Vector3d qInit;
    Eigen::Vector3d qGoal;
    std::vector<std::pair<int, Obstacle3D>> obstacles;
    double xMin, xMax;
    double yMin, yMax;
    double zMin, zMax;

    Problem3D(Eigen::Vector3d qInitIn, Eigen::Vector3d qGoalIn, std::vector<std::pair<int, Obstacle3D>> obstaclesIn, double xMinIn, double xMaxIn, double yMinIn, double yMaxIn, double zMinIn, double zMaxIn){
        qInit = qInitIn;
        qGoal = qGoalIn;
        obstacles = obstaclesIn;
        xMin = xMinIn;
        xMax = xMaxIn;
        yMin = yMinIn;
        yMax = yMaxIn;
        zMin = zMinIn;
        zMax = zMaxIn;
    }
};

struct kinoAgent{
    Eigen::VectorXd states;
    Eigen::VectorXd controls;
    Eigen::VectorXd stateUpperBounds;
    Eigen::VectorXd stateLowerBounds;
    Eigen::VectorXd controlUpperBounds;
    Eigen::VectorXd controlLowerBounds;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    double m;
    double g;
};

double heuristic(Node node, Eigen::Vector3d goal);
double heuristicState(stateNode node, Eigen::Vector3d goal);

class aStar{
public:
    bool success;
    std::vector<Node> path; 
    std::vector<stateNode> statePath;
    double pathCost;

    aStar(bool successIn, std::vector<Node> pathIn, double pathCostIn){
        success = successIn;
        path = pathIn;
        pathCost = pathCostIn;
    }
    aStar(bool successIn, std::vector<stateNode> pathIn, double pathCostIn){
        success = successIn;
        statePath = pathIn;
        pathCost = pathCostIn;
    }

    void search(Problem3D problem, std::vector<Node> nodes);
    void searchState(Problem3D problem, std::vector<stateNode> nodes);
};

aStar rrt(const Problem3D& problem, int n, double r, double epsilon, double p);
aStar kinoRRT(Problem3D& problem, kinoAgent& agent, int& n, double& epsilon, double& p);

Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d>& vertices);
std::vector<Eigen::Vector3d> expandFace(const std::vector<Eigen::Vector3d>& face, double margin);
void expandObstacles(Problem3D& problem, double margin);

void smoothPath(aStar& astar, Problem3D& problem);
void smoothStatePath(aStar& astar, Problem3D& problem, kinoAgent& agent);
Eigen::MatrixXd solveLyapunovNumerically(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double t);
Eigen::MatrixXd integrand(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double t, double tau);