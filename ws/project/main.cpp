#include <iostream>

#include<chrono>

#include "AMPCore.h"
#include "hw/HW4.h"
#include "Helpers.h"
#include "QuadAgentProperties.h"
#include "QuadAgentTools.h"
#include "QuadMultiRRT.h"
#include "QuadOutput.h"

inline bool greaterThan(double u, double v){
    return u<v;
}

std::vector<double> calcQuantiles(std::vector<double> data){
    std::vector<double> quantiles(5);
    int n = data.size();
    std::sort(data.begin(), data.end(), greaterThan);

    for(int i=0; i<data.size(); i++)
        std::cout<<data[i]<<", ";
    std::cout<<"\n";

    quantiles[0] = data[0];

    for(int i=0; i<n; i++){
        if (i == n/4){
            quantiles[1] = data[i];
        }
        if (i == n/2){
            quantiles[2] = data[i];
        }
        if (i == n*3/4){
            quantiles[3] = data[i];
        }
    }

    quantiles[4] = data[data.size()-1];

    return quantiles;
}

std::vector<double> printQuantiles(std::vector<double> data){
    std::vector<double> quantiles = calcQuantiles(data);
    std::cout<<"Quantiles: \n";
    for(int i=0; i<quantiles.size(); i++)
        std::cout<<std::to_string(quantiles[i])<<"\n";

    return quantiles;
} 

QuadAgentProblem setupSimpleSingleAgentProblem(){
    QuadAgentProblem prob;

    prob.env = amp::HW4::getEx3Workspace1();
    prob.env.x_min = -10;
    prob.env.x_max = 10;
    prob.env.y_min = -10;
    prob.env.y_max = 10;

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

std::vector<amp::Polygon> getObstacleWS2(){
    std::vector<Eigen::Vector2d> pg1_verts;
    pg1_verts.push_back(Eigen::Vector2d(-5,-2));
    pg1_verts.push_back(Eigen::Vector2d(-3,-2));
    pg1_verts.push_back(Eigen::Vector2d(-3,0));
    pg1_verts.push_back(Eigen::Vector2d(-5,0));

    std::vector<Eigen::Vector2d> pg2_verts;
    pg2_verts.push_back(Eigen::Vector2d(-1,-1));
    pg2_verts.push_back(Eigen::Vector2d(1,-1));
    pg2_verts.push_back(Eigen::Vector2d(1,1));
    pg2_verts.push_back(Eigen::Vector2d(-1,1));

    std::vector<Eigen::Vector2d> pg3_verts;
    pg3_verts.push_back(Eigen::Vector2d(3,-2));
    pg3_verts.push_back(Eigen::Vector2d(5,-2));
    pg3_verts.push_back(Eigen::Vector2d(5,0));
    pg3_verts.push_back(Eigen::Vector2d(3,0));

    std::vector<Eigen::Vector2d> pg4_verts;
    pg4_verts.push_back(Eigen::Vector2d(2,3));
    pg4_verts.push_back(Eigen::Vector2d(4,3));
    pg4_verts.push_back(Eigen::Vector2d(4,5));
    pg4_verts.push_back(Eigen::Vector2d(2,5));

    std::vector<Eigen::Vector2d> pg5_verts;
    pg5_verts.push_back(Eigen::Vector2d(-2,3));
    pg5_verts.push_back(Eigen::Vector2d(-4,3));
    pg5_verts.push_back(Eigen::Vector2d(-4,5));
    pg5_verts.push_back(Eigen::Vector2d(-2,5));

    std::vector<amp::Polygon> obs;

    obs.push_back(amp::Polygon(pg1_verts));
    obs.push_back(amp::Polygon(pg2_verts));
    obs.push_back(amp::Polygon(pg3_verts));
    obs.push_back(amp::Polygon(pg4_verts));
    obs.push_back(amp::Polygon(pg5_verts));

    return obs;
}

QuadAgentProblem setupSimpleMultiAgentProblem(int num_agents, int ws){
    QuadAgentProblem prob;
    prob.num_agents = num_agents;

    std::vector<Eigen::Vector2d> pg1_verts;
    

    // amp::HW4::getEx3Workspace1()
    switch (ws){
        // one square
        case 1:
        {
            pg1_verts.push_back(Eigen::Vector2d(-1,-1));
            pg1_verts.push_back(Eigen::Vector2d(1,-1));
            pg1_verts.push_back(Eigen::Vector2d(1,1));
            pg1_verts.push_back(Eigen::Vector2d(-1,1));
            amp::Polygon pg1(pg1_verts);
            prob.env.obstacles.push_back(pg1);
            break;
        }
        // 5 squares in olympic logo pattern
        case 2:
        {
            prob.env.obstacles = getObstacleWS2();
            break;
        }
        default:
            break;
    }


    prob.env.x_min = -10;
    prob.env.x_max = 10;
    prob.env.y_min = -10;
    prob.env.y_max = 10;

    std::vector<QuadState> q_inits(5);
    std::vector<QuadState> q_goals(5);

    q_inits[0] << -4, -6, 0,0,0,0;
    q_inits[1] <<  4, -6, 0,0,0,0;
    q_inits[2] << -2, -6, 0,0,0,0;
    q_inits[3] <<  2, -6, 0,0,0,0;
    q_inits[4] <<  0, -6, 0,0,0,0;

    q_goals[0] <<  4,  6, 0,0,0,0;
    q_goals[1] << -4,  6, 0,0,0,0;
    q_goals[2] <<  2,  6, 0,0,0,0;
    q_goals[3] << -2,  6, 0,0,0,0;
    q_goals[4] <<  0,  6, 0,0,0,0;

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


const bool RUN_TESTS = false;
const bool RUN_MISC = true;


int main(int argc, char ** argv){

    if(RUN_TESTS){
        // int num_agents = 1;
        // int ws = 2;
        // QuadAgentProblem prob = setupSimpleMultiAgentProblem(num_agents,ws);

        // int n_rtt = 10;
        // double Dt=0.1;
        // double p_goal = 0.05;
        // double epsilon = 0.5;
        // int n_runs=10;

        // std::vector<double> times(n_runs);
        // int num_success=0;
        // for(int i=0; i<n_runs; i++){
        //     QuadMultiRRT kdrrt(n_rtt, Dt, p_goal, epsilon);

        //     auto start = std::chrono::high_resolution_clock::now();
        
        //     QuadProblemResult plan_result = kdrrt.plan(prob);

        //     auto end = std::chrono::high_resolution_clock::now();

        //     std::chrono::duration<double> duration = end-start;

        //     times[i] = double(duration.count());
        //     num_success += plan_result.success;
        // }

        // std::cout<<"Time quantiles:\n";
        // printQuantiles(times);

        // std::cout<<"number of successes: "<<num_success<<" out of "<< n_rtt<<" runs \n";

        // std::list<std::vector<double>> data_sets = {times};
        // std::vector<std::string> labels = {"one agent ws2 n_rrt = 10000"};
        // std::string title = "";
        // std::string xlabel = "";
        // std::string ylabel = "";
        // amp::Visualizer::makeBoxPlot(data_sets,labels,title,xlabel,ylabel);

        // amp::Visualizer::showFigures();
    } else if(RUN_MISC){
        int num_agents = 1;
        int ws = 1;
        QuadAgentProblem prob = setupSimpleMultiAgentProblem(num_agents,ws);

        int n_rtt = 10000;
        double Dt=0.1;
        double p_goal = 0.05;
        double epsilon = 0.5;
        int n_runs=10;

        Eigen::Matrix<double,4,1> epsilon_vec;
        epsilon_vec << .5, 3.0, 2.0, 1;

        QuadMultiRRT kdrrt(epsilon_vec,n_rtt, Dt, p_goal, epsilon);    
        QuadProblemResult plan_result = kdrrt.plan(prob);

        std::cout<<"Num trajectories: "<< plan_result.paths.size() << "\n";

        amp::MultiAgentPath2D amp_paths = trajectoriesToAmpMultiAgent(plan_result.paths);
        amp::MultiAgentProblem2D amp_prob = quadToAmpMultiProblem(prob);

        std::cout<<"Num amp paths: "<< amp_paths.agent_paths.size() << "\n";
        std::cout<<"Num amp agents: " << amp_prob.agent_properties.size() << "\n";

        auto quad_controls = kdrrt.getControlInputs();
        QuadOutput::writeToFile(prob, prob.agents, plan_result.paths, quad_controls);

        auto spps = kdrrt.getSPPS();
        auto node_map = kdrrt.getNodeMaps();
        std::cout<< "graph size" << spps[0].graph->nodes().size()<<"\n";
        amp::Problem2D prob2d;
        prob2d.obstacles = prob.env.obstacles;
        prob2d.x_min = -10;
        prob2d.x_max = 10;
        prob2d.y_min = -10;
        prob2d.y_max = 10;
        amp::Graph test = *(spps[0].graph);

        for(int i=0; i<num_agents; i++){
            prob2d.q_init = QuadAgentTools::getPos(prob.agents[i].q_init);
            prob2d.q_goal = QuadAgentTools::getPos(prob.agents[i].q_goal);
            amp::Visualizer::makeFigure(prob2d, *(spps[i].graph), node_map[i]);
        }

        amp::Visualizer::makeFigure(amp_prob,amp_paths);
        amp::Visualizer::showFigures();
    }
}