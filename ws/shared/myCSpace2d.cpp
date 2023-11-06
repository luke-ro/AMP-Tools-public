#include "myCSpace2d.h"

void myCSpace2d::constructFromCircleAgent(const amp::Environment2D& env, const amp::CircularAgentProperties& circular_agent){
    Eigen::Vector2d point;
    Eigen::Vector2d near;
    double r = circular_agent.radius;
    for(int i=0; i<_n0; i++){
        point[0] = H::idxToNum(i,_n0,env.x_min,env.x_max);
        for(int j=0; j<_n1; j++){
            point[1] = H::idxToNum(j,_n1,env.y_min,env.y_max);
            near = H::obstaclesClosePt(env,point);
            if((point-near).norm()<r){
                operator()(i,j) = true;
            }
        }
    }
}

std::pair<std::size_t, std::size_t> myCSpace2d::getCellFromPoint(double x0, double x1) const{
    return std::pair(
        H::numToIdx(x0,m_x0_bounds.first,m_x0_bounds.second,_n0),
        H::numToIdx(x1,m_x1_bounds.first,m_x1_bounds.second,_n1)
    );
}
        
/**
 * @brief checks for collisions along a discrete 2d grid. TODO make this not use a sampling method
*/
bool myCSpace2d::freeBtwPoints(Eigen::Vector2d p1, Eigen::Vector2d p2){
    std::pair<int,int> sz = size();

    // get how far to step along line. Divide step by two for good measure?
    double step = 0.5*std::min(double(sz.first)/abs(x0Bounds().first-x0Bounds().second),
                               double(sz.second)/abs(x1Bounds().first-x1Bounds().second));

    // get the number of points to sample based on distance between points
    int n = ceil((p1-p2).norm()/step);
    n = std::max(3,n); // catch all if n is less than 3

    std::vector<Eigen::Vector2d> sample_points = H::linspace2D(p1,p2,n);

    for(auto pt : sample_points){
        if(inCollision(pt[0],pt[1])){
            // std::cout<<"Collision detected myCSpace2D\n";
            return false;
        }
    }

    return true;
}