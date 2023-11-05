// This is the bare min to get a cspace grid. 

#include "AMPCore.h"

#include "Helpers.h"

class myCSpace2d : public amp::GridCSpace2D{
    public:

    myCSpace2d(int s0, int s1, double x0_min, double x0_max, double x1_min, double x1_max):
        GridCSpace2D(s0, s1, x0_min, x0_max, x1_min, x1_max),
        _n0(s0),
        _n1(s1)
    {}

    virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;


    void constructFromCircleAgent(const amp::Environment2D& env, const amp::CircularAgentProperties& circular_agents);

    int _n0;
    int _n1;
};

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
        H::numToIdx(x0,_n0,m_x0_bounds.first,m_x0_bounds.second),
        H::numToIdx(x1,_n1,m_x1_bounds.first,m_x1_bounds.second)
    );
}
        