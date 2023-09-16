#include "MyBugAlgorithm.h"

// returns true if val is between the values specified by x1 and x2 exlusive
bool isBetwOpen(double val, double x1, double x2){
    return (val>std::min(x1,x2)) && (val<std::max(x1,x2));
}

// returns true if val is between the values specified by x1 and x2 inclusive
bool isBetwClosed(double val, double x1, double x2){
    return (val>=std::min(x1,x2)) && (val<=std::max(x1,x2));
}

// returns true if val is between the values specified by x1(inclusive) and x2 (exclusive)
bool isBetwLeftClosed(double val, double x1, double x2){
    if (x1<x2){
        return (val>=x1) && (val<x2);
    }else if(x1>x2){
        return (val>x2) && (val<=x1);
    }else{
        return val==x1;
    }
}


// Implement your methods in the `.cpp` file, for example: BUG1!!!
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem){

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d q = Eigen::Vector2d(problem.q_init);
    // Eigen::Vector2d stepToGoal;
    int i_hit = -1;
    path.waypoints.push_back(q);

    // iterate until path is found or failure.
    while(true){

        //iterate until at goal or in collision
        while(!(atGoal(problem,q) || isCollsion(problem,q+stepToGoal(problem,q)))){
            q += stepToGoal(problem,q);
        }

        // add point to path 
        path.waypoints.push_back(q);
        i_hit = path.waypoints.size()-1;

        //check to see if at goal
        if (atGoal(problem,q)){
            return path;
        }

        //follow boundary
        // 1. get current edge
        // 2. turn left 
        while(!(atGoal(problem,q)|| atPoint(path.waypoints[i_hit],q))){
            // follow boundary
            // while(!(atCorner || isCollsion(q+step))) //may have to invert the while loops
            // 3. go straight until collision or end of edge
                // 4. if  distToGoal(problem,q)< minDist
                    // 5. q_min  = q (gonna be wierd, make a checkpoint and keep modifying it?)
                // 6. if (atGoal(problem,q))
                    // return path
            // 7. path.waypoints.push_back(q)
            // 8. follow new edge (happens for both collision and edge)

        }

        //check to see if at goal
        if(atGoal(problem,q)){
            return path;
        } 

        //follow boundary back to q_li
        // 1. go to q_Li (follow boundary)

        if(isCollsion(problem, q + stepToGoal(problem,q))){
            amp::Path2D failure;
            return failure;
        }
    }
    // path.waypoints.push_back(problem.q_init);
    // path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    // path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    // path.waypoints.push_back(problem.q_goal);

    return path;
}

// bool MyBugAlgorithm::checkCollision(const amp::Problem2D& problem, Eigen::Vector2d x1, Eigen::Vector2d x2, float deltaX){
//     int dim=2;
//     n = ceil(((x1-x2).norm())/deltaX);

//     std::vector<std::vector<double>> points(dim);
//     for(int i=0;i<2;i++){
//         points[i] = helpers::linspace(x1[i],x2[i],n)
//     }

//     for(int p=0;p<n;p++){
//         if insideObstacle2d(points[0][p],points[1][p]){
//             return false;
//         }
//     }

//     return true;
// }

/**
 * Checks entire workspace for collision
 * 
 * @param problem the dataframe that contains the workspace ans obstacles
 * @param x the 2d point to check
*/
bool MyBugAlgorithm::isCollsion(const amp::Problem2D& problem, Eigen::Vector2d x){
    // iterate through obstacles
    for(auto obs : problem.obstacles){
        if (insidePolygon(obs,x)){
            //collision was found
            return true;
        }
    }

    // if at this point, a collision was not found
    return false;
}

/**
 * Checks if a point is within a polygon, edge inclusive. 
 * 
 * @param pg the polygon to check against
 * @param q the point to check
 * @return true if q is inside or on pg, false otherwise
*/
bool MyBugAlgorithm::insidePolygon(const amp::Polygon& pg, const Eigen::Vector2d& q){
        //reset number of intersections for each obstacle
    int num_intersections = 0;

    int num_verts = pg.verticesCCW().size();
    
    Eigen::Vector2d p1,p2;

    //iterate through edges
    for(int i=0;i++;i<num_verts+1){
        // get the current edge's vertices 
        p1 = pg.verticesCCW()[i];
        p2 = pg.verticesCCW()[(i+1)%num_verts]; //gets the next vertex (and wraps to the begining)

        // calculate the line for the edge
        double m = (p2[1]-p1[1])/(p2[0]-p1[0]);
        double x_inter = (q[1]-p1[1])/m + p1[0];
        auto f_of_x = [m,p1](double x){return (m*(x-p1[0]))+p1[1];}; 

        // only checking for intersections on the right of the point
        if (m!=0 && x_inter<q[0]){
            // intersection occurs to the left of q[0]. 
            // Pass.

        //check to see if point lies on the edge. Return true if so.
        }else if(q[1]==f_of_x(q[0]) && isBetwClosed(q[0],p1[0],p2[0])){
            return true;
        
        // slope of zero (horiz) edge condition
        }else if(m==0){
            // horizontal lines dont count as intersection. 
            // Pass.

        // inf slope (vert) edge condition
        }else if(std::isinf(m) || std::isinf(-m)){
            if(q[0] <= p1[0] && isBetwLeftClosed(q[1],p1[1],p2[1])){
                // point is to the left inclusive and is vertically within bounds
                num_intersections++;
            }

        // check the first vertex. If same y (and to the left exclusive), count as intersection
        }else if(q[1]==p1[1] && q[0]>p1[1]){
            //might be hnadled by the last if statement?
            num_intersections++;        
        

        // check if the x intersept is between the two x coords of points
        }else if(isBetwLeftClosed(x_inter,p1[0],p2[0])){
            num_intersections++;
        }
    }

    // odd number of intersections means inside obstacle
    if(num_intersections%2==1)
        return true;
    return false;
}
