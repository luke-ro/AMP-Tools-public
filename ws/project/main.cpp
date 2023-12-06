#include <iostream>

#include "AMPCore.h"
#include "hw/HW4.h"
#include "Helpers.h"
#include "QuadAgent.h"


Eigen::Matrix<double,6,1> rk4(QuadAgent agent, Eigen::Matrix<double,6,1> y0, Eigen::Vector3d u, double dt){
    Eigen::Matrix<double,6,1> k1 = agent.dynamics(y0, u);
    Eigen::Matrix<double,6,1> k2 = agent.dynamics(y0+dt*k1/2, u);
    Eigen::Matrix<double,6,1> k3 = agent.dynamics(y0+dt*k2/2, u);
    Eigen::Matrix<double,6,1> k4 = agent.dynamics(y0+dt*k3, u);

    Eigen::Matrix<double,6,1> y1 = y0 + (dt/6) * (k1 + (2*k2) + (2*k3) + k4);
    return y1;
}

int main(int argc, char ** argv){
    QuadAgent agent;
    Eigen::Vector3d u;
    Eigen::Matrix<double,6,1> test = rk4(agent, agent.getState(), u, 0.1);
    for(int i=0; i<6; i++){
        std::cout<<test(i)<<"\n";
    }
}