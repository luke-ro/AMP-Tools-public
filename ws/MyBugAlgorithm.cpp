#include "MyBugAlgorithm.h"

// returns true if val is between the values specified by x1 and x2
bool isBetween(val,x1,x2){
    return (val>std::min(x1,x2)) && (val<std::max(x1,x2));
}

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

bool MyBugAlgorithm::insideObstacles(const amp::Problem2D& problem, Eigen::vector2d x){
    // predefine some variables
    Eigen::Vector2d p1;
    Eigen::Vector2d p2;
    int num_verts;
    int num_intersections;

    // iterate through obstacles
    for(auto obs : problem){


    }
    return false;
}

bool MyBugAlgorithm::insidePolygon(amp::Polygon pg, Eigen::Vector2d q){
        //reset number of intersections for each obstacle
    int num_intersections = 0;

    int num_verts = pg.verticesCCW.size();
    
    //iterate through edges
    for(int i=0;i++;i<num_verts+1){
        // get the current edge's vertices 
        p1 = obs[i];
        p2 = obs[(i+1)%num_verts]; //gets the next vertex (and wraps to the begining)


        // slope of zero (horiz) edge condition
        if((p2[1]-p1[1])==0){
            // otherwise horizontal lines dont count as intersection. Do nothing.

        // check the first vertex. If same y (and to the left exclusive), count as intersection
        }else if(q[1]==p1[1] && q[0]>p1[1]){
            num_intersections++;        

        // inf slope (vert) edge condition
        }else if((p2[0]-p1[0])==0){
            if(p2[0] == q[0]){
                // point is to the right of the line. Do nothing. 

            }else if(isBetween(q[1],p1[1],p2[1])){
                // point is to the left and is vertically within other bounds
                num_intersections++;
            }
            continue;
        }

        // calculate the line for the edge
        double m = (p2[1]-p1[1])/(p2[0]-p1[0]);
        double x_inter = (q[1]-p1[1])/m + p[0];

        // only checking for intersections on the right of the point
        if (x_inter<q[0]){
            //intersection occurs to the left of q[0]. continue.
            continue;
        }

        //
        if(isBetween(x_inter,p1[0],p2[0])){
            num_intersections++;
        }
    }

    // odd number of intersections means inside obstacle
    if(num_intersections%2==1)
        return true;
}
