#include <iostream>

#include "AMPCore.h"
#include "hw/HW4.h"
#include "Helpers.h"
#include "QuadAgentProperties.h"
#include "QuadAgentTools.h"
#include "QuadMultiRRT.h"
#include "QuadOutput.h"

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

QuadAgentProblem setupSimpleMultiAgentProblem(int num_agents){
    QuadAgentProblem prob;
    prob.num_agents = num_agents;

    prob.env = amp::HW4::getEx3Workspace1();

    std::vector<QuadState> q_inits(5);
    std::vector<QuadState> q_goals(5);

    q_inits[0] << -2,2,0,0,0,0;
    q_inits[1] << -2,-1,0,0,0,0;
    q_inits[2] << -2,0,0,0,0,0;
    q_inits[3] << -2,1,0,0,0,0;
    q_inits[4] << -2,2,0,0,0,0;

    q_goals[0] << -2,-2,0,0,0,0;
    q_goals[1] << 2,1,0,0,0,0;
    q_goals[2] << 2,0,0,0,0,0;
    q_goals[3] << 2,-1,0,0,0,0;
    q_goals[4] << 2,-2,0,0,0,0;

    for(int i=0; i<num_agents; i++){
        QuadAgentProperties agent(q_inits[i],q_goals[i]);
        prob.agents.push_back(agent);
    }

    return prob;
}

amp::MultiAgentPath2D trajectoriesToAmpMultiAgent(QuadAgentsTrajectories quad_trajs){
    int n = quad_trajs.size();
    amp::MultiAgentPath2D amp_paths(n);

    std::cout<< "number of paths in trajectoriesTo... : "<< n<< "\n";

    for(int i=0; i<n; i++){
        amp::Path2D path;
        for(auto x : quad_trajs[i]){
            path.waypoints.push_back(Eigen::Vector2d(x(0),x(1)));
        }
        amp_paths.agent_paths[i] = path;
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
    QuadAgentProblem prob = setupSimpleMultiAgentProblem(1);
    QuadMultiRRT kdrrt(300,.1);
    QuadAgentsTrajectories quad_trajectories = kdrrt.plan(prob);

    std::cout<<"Num trajectories: "<< quad_trajectories.size() << "\n";

    amp::MultiAgentPath2D amp_paths = trajectoriesToAmpMultiAgent(quad_trajectories);
    amp::MultiAgentProblem2D amp_prob = quadToAmpMultiProblem(prob);

    std::cout<<"Num amp paths: "<< amp_paths.agent_paths.size() << "\n";
    std::cout<<"Num amp agents: " << amp_prob.agent_properties.size() << "\n";

    QuadOutput::writeToFile(quad_trajectories);

    auto spps = kdrrt.getSPPS();
    auto node_map = kdrrt.getNodeMaps();
    std::cout<< "graph size" << spps[0].graph->nodes().size()<<"\n";
    amp::Problem2D prob2d;
    prob2d.obstacles = prob.env.obstacles;
    prob2d.x_min = -5;
    prob2d.x_max = 5;
    prob2d.y_min = -5;
    prob2d.y_max = 5;
    amp::Graph test = *(spps[0].graph);
    amp::Visualizer::makeFigure(prob2d, test, node_map[0]);
    amp::Visualizer::showFigures();
    // amp::Visualizer::makeFigure(amp_prob,amp_paths);
}