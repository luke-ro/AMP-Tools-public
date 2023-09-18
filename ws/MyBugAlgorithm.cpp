#include "MyBugAlgorithm.h"
#include "stdio.h"

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
    Eigen::Vector2d q_last, temp;
    // Eigen::Vector2d stepToGoal;
    int i_hit = -1;
    path.waypoints.push_back(q);

    // iterate until path is found or failure.
    while(true){

        //iterate until at goal or in collision
        while(!(atGoal(problem,q) || isCollsion(problem,q+stepToGoal(problem,q)))){
            q_last = q;
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
        int i = 0; //counter to let the bug travel a bit
        do{
            // follow boundary
            // while(!(atCorner || isCollsion(q+step))) //may have to invert the while loops
            // 3. go straight until collision or end of edge
                // 4. if  distToGoal(problem,q)< minDist
                    // 5. q_min  = q (gonna be wierd, make a checkpoint and keep modifying it?)
                // 6. if (atGoal(problem,q))
                    // return path
            // 7. path.waypoints.push_back(q)
            // 8. follow new edge (happens for both collision and edge)
            temp = q;
            q += borderFollowLeft(problem,q,q_last);
            q_last = temp;
            path.waypoints.push_back(q);
            i++;

        }
        while(!(atGoal(problem,q)|| ((atPoint(path.waypoints[i_hit],q)) && i>2)));

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
            // LOG("COLLISION: "<< std::format("{}",x[0]) << ", " << std::format("{}",x[1]));
            // printf("COLLISION AT %2.2f, %2.2f", x[0],x[1]);
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
    // printf("`insidePolygon` AT %2.2f, %2.2f", q[0],q[1]);
    //iterate through edges
    for(int i=0;i<num_verts+1;i++){
        // get the current edge's vertices 
        p1 = pg.verticesCCW()[i%num_verts];
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

/**
 *  Follows an obstacle(s) clockwise (left turning)
 * 
 * @param problem the workspace
 * @param q the current location
 * @param q_prev previous steps location
 * @return a step along the border
*/
Eigen::Vector2d MyBugAlgorithm::borderFollowLeft(const amp::Problem2D& problem, const Eigen::Vector2d& q, const Eigen::Vector2d& q_prev){
    //try to step in current direction...
        //check for obstacles
    Eigen::Vector2d dir = (q-q_prev).normalized();
    double angle = getAngle(dir);

    Eigen::Vector2d right_vec {0,-_epsilon};
    right_vec = rotateVec(right_vec, angle);

    Eigen::Vector2d forw_vec {_epsilon,0};
    forw_vec = rotateVec(forw_vec, angle);

    // checks if a step in the same direction results in the righ point being in the obstacle 
    // and the foward point being free
    bool foward_free = !isCollsion(problem, q+forw_vec);
    bool right_collision = isCollsion(problem, q+forw_vec+right_vec);
    
    if(foward_free && !right_collision)
        std::cout<<"------------------------THE EDGE OF THE OBSTACLE HAS BEEN LOST"<<"\n";
    
    if(foward_free && right_collision){
        return forw_vec;

    // if foward collision:
        // turn left until foward is free
    }else if(!foward_free){
        do{
            forw_vec = rotateVec(forw_vec,D_theta);
            right_vec = rotateVec(right_vec,D_theta);
        }while(isCollsion(problem,q+forw_vec));

    // else if right is free
        // turn right until right is in collision 
    }else if(!right_collision){
        do{
            forw_vec = rotateVec(forw_vec,-D_theta);
            right_vec = rotateVec(right_vec,-D_theta);
        }while(!isCollsion(problem,q+forw_vec+right_vec));
    }


    return forw_vec;
}

/**
 * Rotates a vector by a given angle (keeps it in the same frame)
 * 
 * @param vec vector to rotate
 * @param ang angle to rotate by in rads
*/
Eigen::Vector2d MyBugAlgorithm::rotateVec(Eigen::Vector2d vec, double ang){
    Eigen::Matrix<double, 2, 2> dcm;
    dcm << cos(ang),-sin(ang),
           sin(ang), cos(ang);

    return dcm*vec;
}