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
/**
 * Finds a path through the environment using Bug 1 algorithm
 * 
 * @param problem amp::Problem2D describing the goal points and worspace.
 * @return path from q_start to q_goal
*/
amp::Path2D MyBugAlgorithm::planBug2(const amp::Problem2D& problem){

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d q = Eigen::Vector2d(problem.q_init);
    Eigen::Vector2d q_last, temp;
    // Eigen::Vector2d stepToGoal;
    int i_hit = -1;
    int i_min = 0;
    double dist_min=1000000000000000000;
    path.waypoints.push_back(q);

    // iterate until path is found or failure.
    int loops = 0;
    while(true){

        //iterate until at goal or in collision
        while(!(atGoal(problem,q) || isCollsion(problem,q+stepToGoal(problem,q)))){
            q_last = q;
            q += stepToGoal(problem,q);
        }

        // add point to path 
        path.waypoints.push_back(q);
        i_hit = path.waypoints.size()-1;
        i_min = path.waypoints.size()-1;
        dist_min = distToGoal(problem,q);

        //check to see if at goal
        if (atGoal(problem,q)){
            return path;
        }

        //follow boundary
        int i = 0; //counter to let the bug travel a bit
        do{
            // below is code that gets rid of the temp stuff, but is slower?
            // q += borderFollowLeft(problem,q,path.waypoints[path.waypoints.size()-1]);
            // path.waypoints.push_back(q);

            // Need to keep track of last q 
            temp = q;
            q += borderFollowLeft(problem,q,q_last);
            q_last = temp;
            path.waypoints.push_back(q);

            double dist = distToGoal(problem,q);
            if (dist<dist_min){
                i_min = path.waypoints.size()-1;
                dist_min = dist;
            }
            i++;

        }
        while(!(atGoal(problem,q)|| ((atPoint(path.waypoints[i_hit],q)) && i>2))); //i>2 gets the bug away from the hit point

        //check to see if at goal
        if(atGoal(problem,q)){
            return path;
        } 

        // backtrack the shorter distance around obstacle
        if (pathDistane(path,path.waypoints.size()-1,i_min) <= pathDistane(path,i_hit,i_min)){
            //backtrack along path
            for(int k=path.waypoints.size()-1; k>=i_min; k--){
                path.waypoints.push_back(path.waypoints[k]);
            }
            q = path.waypoints[path.waypoints.size()-1];
        }else{
            Eigen::Vector2d q_hit = path.waypoints[i_hit];
            // go around obstacle in same direction
            do{
                temp = q;
                q += borderFollowLeft(problem,q,q_last);
                q_last = temp;
                path.waypoints.push_back(q);
            }while(!atPoint(q,q_hit));
        }
        //follow boundary back to q_li
        // 1. go to q_Li (follow boundary)

        if(isCollsion(problem, q + stepToGoal(problem,q))){
            amp::Path2D failure;
            return failure;
        }
        // if(++loops>0) break;

    }
    // path.waypoints.push_back(problem.q_init);
    // path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    // path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    // path.waypoints.push_back(problem.q_goal);

    return path;
}

/**
 * Finds a path through the environment using Bug 2 algorithm
 * 
 * @param problem amp::Problem2D describing the goal points and worspace.
 * @return path from q_start to q_goal
*/
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem){

    setMline(problem);

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d q = Eigen::Vector2d(problem.q_init);
    Eigen::Vector2d q_last, temp;
    // Eigen::Vector2d stepToGoal;
    int i_hit = -1;
    int i_min = 0;
    double dist_min=1000000000000000000;
    path.waypoints.push_back(q);

    // iterate until path is found or failure.
    int loops = 0;
    while(true){

        //iterate until at goal or in collision
        while(!(atGoal(problem,q) || isCollsion(problem,q+stepToGoal(problem,q)))){
            q_last = q;
            q += stepToGoal(problem,q);
        }

        // add point to path 
        path.waypoints.push_back(q);
        i_hit = path.waypoints.size()-1;
        i_min = path.waypoints.size()-1;
        dist_min = distToGoal(problem,q);
        double dist_hit = dist_min;

        //check to see if at goal
        if (atGoal(problem,q)){
            return path;
        }

        //follow boundary
        int i = 0; //counter to let the bug travel a bit
        do{
            // below is code that gets rid of the temp stuff, but is slower?
            // q += borderFollowLeft(problem,q,path.waypoints[path.waypoints.size()-1]);
            // path.waypoints.push_back(q);

            // Need to keep track of last q 
            temp = q;
            q += borderFollowLeft(problem,q,q_last);
            q_last = temp;
            path.waypoints.push_back(q);

            double dist = distToGoal(problem,q);
            if (dist<dist_min){
                i_min = path.waypoints.size()-1;
                dist_min = dist;
            }
            i++;

            //mline condition
            if(onMline(q) && dist < (dist_hit+_epsilon) && !isCollsion(problem, q+stepToGoal(problem,q))){
                break;
            }

        }
        while(!(atGoal(problem,q) || ((atPoint(path.waypoints[i_hit],q)) && i>2))); //i>2 gets the bug away from the hit point

        //check to see if at goal
        if(atGoal(problem,q)){
            return path;
        } 

        // backtrack the shorter distance around obstacle
        // if (pathDistane(path,path.waypoints.size()-1,i_min) <= pathDistane(path,i_hit,i_min)){
        //     //backtrack along path
        //     for(int k=path.waypoints.size()-1; k>=i_min; k--){
        //         path.waypoints.push_back(path.waypoints[k]);
        //     }
        //     q = path.waypoints[path.waypoints.size()-1];
        // }else{
        //     Eigen::Vector2d q_hit = path.waypoints[i_hit];
        //     // go around obstacle in same direction
        //     do{
        //         temp = q;
        //         q += borderFollowLeft(problem,q,q_last);
        //         q_last = temp;
        //         path.waypoints.push_back(q);
        //     }while(!atPoint(q,q_hit));
        // }
        //follow boundary back to q_li
        // 1. go to q_Li (follow boundary)

        if(isCollsion(problem, q + stepToGoal(problem,q))){
            amp::Path2D failure;
            return path;
        }
        // if(++loops>0) break;

    }
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
    for(int i=0;i<num_verts;i++){
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
        // }else if(m==0){
        //     // horizontal lines dont count as intersection. 
        //     // Pass.

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
    
    //set a flag if the bug loses the wall
    bool flag = false;
    if(foward_free && !right_collision){
        flag = true; //wall is lost prior to rotation of the follower
    }

    // need a counter in case there is a failure with a whole rotation
    int iterations = ceil(2.0*3.1415/D_theta)+1;
    
    if(foward_free && right_collision){
        return forw_vec;

    // if foward collision:
        // turn left until foward is free
    }else if(!foward_free){
        int i = 0;
        do{
            forw_vec = rotateVec(forw_vec,D_theta);
            right_vec = rotateVec(right_vec,D_theta);
        }while(isCollsion(problem,q+forw_vec) && i++<iterations);
        if(flag && i<iterations) 
            flag = false;

    // else if right is free
        // turn right until right is in collision 
    }else if(!right_collision){
        int i=0;
        do{
            forw_vec = rotateVec(forw_vec,-D_theta);
            right_vec = rotateVec(right_vec,-D_theta);
        }while(!isCollsion(problem,q+forw_vec+right_vec) && i++<iterations);
        if(flag && i<iterations) 
            flag = false;
    }

    if(flag){
        std::cout<<"------------------------THE EDGE OF THE OBSTACLE HAS BEEN LOST"<<"\n";
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

/**
 * Measures the arc length of a path from index i_start to i_end
 * 
 * @param path a amp::Path2D object containing waypoints\
 * @param i_start index of start
 * @param i_end index of end
 * @return length of path between the points
*/
double MyBugAlgorithm::pathDistane(const amp::Path2D& path, int i_start, int i_end){
    double dist=0;
    for(int i=i_start; i<i_end; i++){
        // Eigen::Vector2d r = (path.waypoints[i]-path.waypoints[i+1]).norm();
        dist+=(path.waypoints[i]-path.waypoints[i+1]).norm();
    }
    return dist;
}

/**
 * Sets the variables for the mline
*/
bool MyBugAlgorithm::setMline(const amp::Problem2D& problem){
    _m_slope = (problem.q_goal[1]-problem.q_init[1])/(problem.q_goal[0]-problem.q_init[0]);
    _m_xp = problem.q_init[0];
    _m_yp = problem.q_init[1];
    return true;
}

/**
 * determines if a point is within _epsilon of the mline
*/
bool MyBugAlgorithm::onMline(const Eigen::Vector2d& q){
    return abs(q[1]-mLine(q[0])) < (_epsilon);
}


// ?? MIGHT NOT NEED THIS ??
/**
 * Takes a step along the mline
 * 
 * @param q current point
 * @return a step of length _epsilon along the mline
*/
// Eigen::Vector2d MyBugAlgorithm::stepOnMline(const Eigen::Vector2d& q){
//     Eigen::Vector2d step = {_epsilon,0};
// }