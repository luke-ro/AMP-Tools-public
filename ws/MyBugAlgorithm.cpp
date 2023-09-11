#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) const {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d curr = Eigen::Vector2d(problem.q_init);
    while(true){
        while(!(hit || obstacle encountered)){
            move deltaX foward
        }
        if (at goal){
            return path
        }
        while(!(at goal || at last hit point)){
            follow boundary
            keep track of closest point to goal q_Li
        }

        if(at goal){
            return path
        } 

        go to q_Li (follow boundary)

        if(path to q_goal is blocked){
            return with failure
        }
    }
    // path.waypoints.push_back(problem.q_init);
    // path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    // path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    // path.waypoints.push_back(problem.q_goal);



    return path;
}

bool MyBugAlgorithm::checkCollision(const amp::Problem2D& problem, Eigen::vector2d x1, Eigen::vector2d x2, float deltaX){
    int dim=2;
    n = ceil(((x1-x2).norm())/deltaX);

    std::vector<std::vector<double>> points(dim);
    for(int i=0;i<2;i++){
        points[i] = helpers::linspace(x1[i],x2[i],n)
    }

    for(int p=0;p<n;p++){
        if insideObstacle2d(points[0][p],points[1][p]){
            return false;
        }
    }

    return true;
}