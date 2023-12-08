#include <iostream>

#include "AMPCore.h"
#include "hw/HW4.h"
#include "Helpers.h"
#include "QuadAgentProperties.h"
#include "QuadAgentTools.h"
#include "QuadMultiRRT.h"

QuadAgentProblem setupSimpleSingleAgentProblem(){
    QuadAgentProblem prob;

    prob.env = amp::HW4::getEx3Workspace1();

    // setup agent
    QuadState q_init;
    q_init << -2,-2,0,0,0,0;
    QuadState q_goal;
    q_goal << 2,2,0,0,0,0;
    QuadAgentProperties agent(q_init,q_goal);
    
    prob.num_agents = 1;
    prob.agents.push_back(agent);

    return prob;
}

amp::MultiAgentPath2D trajectoriesToAmpMultiAgent(QuadAgentsTrajectories quad_trajs){
    int n = quad_trajs.size();
    amp::MultiAgentPath2D amp_paths(n);

    for(int i=0; i<n; i++){
        amp::Path2D path;
        for(auto x : quad_trajs[i]){
            path.waypoints.push_back(Eigen::Vector2d(x(0),x(1)));
        }
        amp_paths.agent_paths.push_back(path);
    }
    return amp_paths;
}

amp::MultiAgentProblem2D quadToAmpMultiProblem(QuadAgentProblem quad_prob){
    amp::MultiAgentProblem2D amp_prob;
    amp_prob.x_min = quad_prob.env.x_min;
    amp_prob.x_max = quad_prob.env.x_max;
    amp_prob.y_min = quad_prob.env.y_min;
    amp_prob.y_max = quad_prob.env.y_max;
    amp_prob.obstacles = quad_prob.env.obstacles;

    int num_agents = quad_prob.agents.size();
    for(int i=0; i<num_agents; i++){
        amp::CircularAgentProperties amp_agent;
        amp_agent.radius = quad_prob.agents[i].radius;
        amp_agent.q_init = QuadAgentTools::getPos(quad_prob.agents[i].q_init);
        amp_agent.q_goal = QuadAgentTools::getPos(quad_prob.agents[i].q_goal);
        amp_prob.agent_properties.push_back(amp_agent);
    }

    return amp_prob;
}

int main(int argc, char ** argv){
    QuadAgentProblem prob = setupSimpleSingleAgentProblem();
    QuadMultiRRT kdrrt;
    QuadAgentsTrajectories quad_trajectories = kdrrt.plan(prob);
    amp::MultiAgentPath2D amp_paths = trajectoriesToAmpMultiAgent(quad_trajectories);

    amp::MultiAgentProblem2D amp_prob = quadToAmpMultiProblem(prob);
    amp::Visualizer::makeFigure(amp_prob,amp_paths);
}