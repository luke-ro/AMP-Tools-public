#pragma once
#include "AMPCore.h"
#include "Rotate.h"

namespace H{

    // returns true if val is between the values specified by x1 and x2 exlusive
    inline bool isBetwOpen(double val, double x1, double x2);

    // returns true if val is between the values specified by x1 and x2 inclusive
    inline bool isBetwClosed(double val, double x1, double x2);

    // returns true if val is between the values specified by x1(inclusive) and x2 (exclusive)
    inline bool isBetwLeftClosed(double val, double x1, double x2);

    inline bool insidePolygon(const amp::Polygon& pg, const Eigen::Vector2d& q);

    /**
     * @brief Checks entire workspace for collision
    */
    inline bool checkCollsionEnv(const amp::Environment2D& env, const Eigen::Vector2d& q);

    /*
    *  MATLAB or Numpy linspace like function. 
    *  taken from https://stackoverflow.com/questions/27028226/python-linspace-in-c
    */
    template<typename T>
    inline std::vector<double> linspace(T start_in, T end_in, int num_in);


    inline std::vector<Eigen::Vector2d> linspace2D(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, int n);


    /**
     * @brief gets index of closest vertex
    */
    inline int closestVertex(amp::Polygon pg, Eigen::Vector2d q);

    /**
     * @brief Gets the distance to an polygons closest point.
    */
    inline Eigen::Vector2d pgNearestPt(amp::Polygon pg, const Eigen::Vector2d& q);


    /**
     * @brief returns if a point is to the 'left' of an infinite line defined by two points 
     * (left is 90 deg CCW of the direction defined by p1 and p2)
     * closestVertex
    */
    inline bool isLeftOfLine(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q);

    /**
     * @brief returns the closest point on an obstacle to point q
    */
    inline Eigen::Vector2d obstaclesClosePt(const amp::Environment2D& env, const Eigen::Vector2d& q);

    ///@brief continuous number to index
    inline int numToIdx(double num, double num_min, double num_max, int len_arr)
    {return num/(num_max-num_min)*len_arr;}

    ///@brief index to continous number
    inline double idxToNum(int idx, int len_arr, double cont_min, double cont_max)
    {return double(idx)/len_arr*(cont_max-cont_min);}

    ///@brief returns a random collision free point within the environment bounds
    inline Eigen::Vector2d randomValidSample(const amp::Problem2D& problem, int n=20);

    inline Eigen::Vector2d randomSample(const amp::Problem2D& problem);

    inline bool freeBtwPoints(const amp::Problem2D& problem,const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, int n);

    inline Eigen::Vector2d noise2d(double max);
}




// returns true if val is between the values specified by x1 and x2 exlusive
inline bool H::isBetwOpen(double val, double x1, double x2){
    return (val>std::min(x1,x2)) && (val<std::max(x1,x2));
}

// returns true if val is between the values specified by x1 and x2 inclusive
inline bool H::isBetwClosed(double val, double x1, double x2){
    return (val>=std::min(x1,x2)) && (val<=std::max(x1,x2));
}

// returns true if val is between the values specified by x1(inclusive) and x2 (exclusive)
inline bool H::isBetwLeftClosed(double val, double x1, double x2){
    if (x1<x2){
        return (val>=x1) && (val<x2);
    }else if(x1>x2){
        return (val>x2) && (val<=x1);
    }else{
        return val==x1;
    }
}

inline bool H::insidePolygon(const amp::Polygon& pg, const Eigen::Vector2d& q){
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
 * Checks entire workspace for collision
 * 
 * @param env the dataframe that contains the workspace ans obstacles
 * @param q the 2D point to check
*/
inline bool H::checkCollsionEnv(const amp::Environment2D& env, const Eigen::Vector2d& q){
    // iterate through obstacles
    for(auto obs : env.obstacles){
        if (insidePolygon(obs,q)){
            //collision was found
            // LOG("COLLISION: "<< std::format("{}",x[0]) << ", " << std::format("{}",x[1]));
            // printf("COLLISION AT %2.2f, %2.2f", x[0],x[1]);
            return true;
        }
    }

    // if at this point, a collision was not found
    return false;
}

/*
*  MATLAB or Numpy linspace like function. 
*  taken from https://stackoverflow.com/questions/27028226/python-linspace-in-c
*/
template<typename T>
inline std::vector<double> H::linspace(T start_in, T end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}

inline std::vector<Eigen::Vector2d> H::linspace2D(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, int n){
    std::vector<double> x1 = linspace(p1[0],p2[0],n);
    std::vector<double> x2 = linspace(p1[1],p2[1],n);

    std::vector<Eigen::Vector2d> line;
    Eigen::Vector2d temp;
    
    for(int i=0; i<n; i++){
        temp[0] = x1[i];
        temp[1] = x2[i];
        line.push_back(temp);
    }
    return line;
}


/**
 * @brief returns the closest point on an obstacle to point q
 * 
 * @param
 * @param
 * @return 
*/
inline Eigen::Vector2d H::obstaclesClosePt(const amp::Environment2D& env, const Eigen::Vector2d& q){
    Eigen::Vector2d min_pt {std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()};
    double min_mag {std::numeric_limits<double>::infinity()};

    Eigen::Vector2d cur_point;
    double cur_mag;
    // iterate through obstacles
    for(auto pg : env.obstacles){
        cur_point = pgNearestPt(pg,q);
        cur_mag = (cur_point-q).norm();
        if(cur_mag<min_mag){
            min_pt = cur_point;
            min_mag = cur_mag;
        }
    }
    return min_pt;
}

/**
 * @brief gets index of closest vertex
*/
inline int H::closestVertex(amp::Polygon pg, Eigen::Vector2d q){
    int idx = 0;
    double min_dist = std::numeric_limits<double>::infinity();
    double cur_dist;
    for(int i=0;i<pg.verticesCCW().size();i++){
        cur_dist = (pg.verticesCCW()[i]-q).norm();
        if(cur_dist<min_dist){
            min_dist = cur_dist;
            idx = i;
        }
    }
    return idx;
}

/**
 * @brief Gets the distance to an polygons closest point. Assumes query point is in free space!
 * 
 * @param pg polygon to check 
 * @param q point to find closest point to 
 * @return 2d coordinates of closest point on pg
*/
inline Eigen::Vector2d H::pgNearestPt(amp::Polygon pg, const Eigen::Vector2d& q){
    // get the closest vertex
    int v_close = closestVertex(pg,q);
    int v_left;
    int v_right;

    if(v_close==0){
        v_left = pg.verticesCCW().size()-1;
        v_right = 1;
    }else if(v_close==(pg.verticesCCW().size()-1)){
        v_left = 0;
        v_right = pg.verticesCCW().size()-2;
    }else{
        v_left = v_close-1;
        v_right = v_close+1;
    }

    // std::cout<<v_left<<"\n";
    double ang_close = Rotate::ang(pg.verticesCCW()[v_close],q);
    double ang_left = Rotate::ang(pg.verticesCCW()[v_close],pg.verticesCCW()[v_left]);
    double ang_right = Rotate::ang(pg.verticesCCW()[v_close],pg.verticesCCW()[v_right]);

    if(abs(Rotate::angleDifference(ang_close,ang_left)) > 3.1415/2.0 && abs(Rotate::angleDifference(ang_close,ang_right)) > 3.1415/2.0){
        return pg.verticesCCW()[v_close];
    }


    // Rotate q about the v_close vertex by -ang of the line its closest to, 
    // then the x coord should be the intersection distance along the line.
    // Then rotate back to line that intersection lies on and add to get point along line

    //can probably just do the following rotation with a cos of the angle between the lines
    // |q-vertex|*cos(angle between)

    double dist_along_edge {0};
    Eigen::Vector2d rq {0,0};
    Eigen::Vector2d pt_close{0,0};
    if(abs(Rotate::angleDifference(ang_close,ang_left))<0.5*3.1415 
        && abs(Rotate::angleDifference(ang_close,ang_left))
                <abs(Rotate::angleDifference(ang_close,ang_right))){

        rq = Rotate::rotatePoint(q,-ang_left,pg.verticesCCW()[v_close]);
        rq[0] -= pg.verticesCCW()[v_close][0]; 
        rq[1] = 0.0;
        pt_close = pg.verticesCCW()[v_close] + Rotate::rotatePoint(rq,ang_left,Eigen::Vector2d(0,0));
    }else if(abs(Rotate::angleDifference(ang_close,ang_right))<0.5*3.1415){
        rq = Rotate::rotatePoint(q,-ang_right,pg.verticesCCW()[v_close]);
        rq[0] -= pg.verticesCCW()[v_close][0]; 
        rq[1] = 0.0;
        pt_close = pg.verticesCCW()[v_close] + Rotate::rotatePoint(rq,ang_right,Eigen::Vector2d(0,0));
    }else{
        std::cout<<"H::pgNearestPt: Bad things have happened. Program should not have gotten here."<<"\n";
    }

    return pt_close;
}


/**
 * @brief returns if a point is to the 'left' of an infinite line defined by two points 
 * (left is 90 deg CCW of the direction defined by p1 and p2)
 * closestVertex
 * @param p1 first point
 * @param p1 second point
 * @param q query point
 * @return true if q is to the left or on line
*/
inline bool H::isLeftOfLine(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q){
    // calculate the line for the edge
    double m = (p2[1]-p1[1])/(p2[0]-p1[0]);
    auto f_of_x = [m,p1](double x){return (m*(x-p1[0]))+p1[1];}; 

    if(p1[0]>p2[0]){
        return q[0] <= f_of_x(q[0]);

    } else if(p1[0] < p2[0]){
        return q[0] >= f_of_x(q[0]);

    }else if(p1[1] < p2[1]){
        return q[0] >= p1[0];

    }else if(p1[1] > p2[1]){
        return q[0] <= p1[0];
    }

    //should never get to this line
    std::cout << "isLeftOfLine: completed if stmt with no return. Are p1 & p2 the same point?"<<"\n"; 
    return false;
}

/**
 * 
*/
inline Eigen::Vector2d H::randomValidSample(const amp::Problem2D& problem, int n){
    Eigen::Vector2d pt;
    for(int i=0;i<n;i++){
        pt[0] = (((double)rand()/(double)RAND_MAX)*(problem.x_max-problem.x_min)) + problem.x_min;
        pt[1] = (((double)rand()/(double)RAND_MAX)*(problem.y_max-problem.y_min)) + problem.y_min;
        if(!checkCollsionEnv(problem,pt)){
            return pt;
        }
    }
}

inline Eigen::Vector2d H::randomSample(const amp::Problem2D& problem){
    Eigen::Vector2d pt;
    pt[0] = (((double)rand()/(double)RAND_MAX)*(problem.x_max-problem.x_min)) + problem.x_min;
    pt[1] = (((double)rand()/(double)RAND_MAX)*(problem.y_max-problem.y_min)) + problem.y_min;
    return pt;
}

inline bool H::freeBtwPoints(const amp::Problem2D& problem,const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, int n){
    for(auto p : linspace2D(p1,p2,n)){
        if(checkCollsionEnv(problem,p)){
            return false;
        }
    }

    return true;
}

inline Eigen::Vector2d H::noise2d(double max){
    Eigen::Vector2d vec;
    vec[0] = max*(((double)rand()/(double)RAND_MAX))-(0.5*max);
    vec[1] = max*(((double)rand()/(double)RAND_MAX))-(0.5*max);
    return vec;
}