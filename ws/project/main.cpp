#include <iostream>

#include "AMPCore.h"
#include "hw/HW4.h"
#include "Helpers.h"
#include "QuadAgentProperties.h"
#include "QuadAgentTools.h"


int main(int argc, char ** argv){
    QuadState quad_traj;
    quad_traj<<0,0,0,0,0,0;

    QuadAgentProperties agent;
    Eigen::Vector2d u;
    QuadDerivativeState test = QuadAgentTools::rk4(agent, quad_traj, u, 0.1);
    for(int i=0; i<6; i++){
        std::cout<<test(i)<<"\n";
    }
}