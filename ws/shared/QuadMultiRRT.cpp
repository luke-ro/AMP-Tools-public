#include "QuadMultiRRT.h"

Eigen::Vector2d getPos(const QuadState& state){
    return Eigen::Vector2d(state(0),state(1));
}

bool stepFreeAtTime(const QuadAgentProblem& prob, const QuadAgentsTrajectories& paths, const QuadState& q1, const QuadState& q2, int idx_agent, int idx_time){
    // the agent being checked
    Eigen::Vector2d q1_agent2 = getPos(q1);
    Eigen::Vector2d q2_agent2 = getPos(q2);

    //loop through agents that already have plans
    for(int k=0;k<idx_agent; k++){
        int n = 10;
        double min_r = 1.5*(prob.agents[idx_agent].radius + prob.agents[k].radius);

        //points for higher priority agent (agents with plans)
        Eigen::Vector2d q1_agent1;
        Eigen::Vector2d q2_agent1;
        
        // check to make sure that the agent was moving at that time. 
        int sz_agent1 = paths[k].size();
        if(sz_agent1-1<idx_time){
            q1_agent1 = getPos(paths[k][sz_agent1-1]);
            q2_agent1 = getPos(paths[k][sz_agent1-1]);
        }else{
            q1_agent1 = getPos(paths[k][idx_time-1]);
            q2_agent1 = getPos(paths[k][idx_time]);
        }


        std::vector<Eigen::Vector2d> path_agent1  = H::linspace2D(q1_agent1, q2_agent1,n);
        std::vector<Eigen::Vector2d> path_agent2  = H::linspace2D(q1_agent2, q2_agent2,n);

        for(int i = 0; i<n; i++){
            if((path_agent1[i]-path_agent2[i]).norm() < min_r){
                // std::cout<<"failed stepFreeAtTime: "<<q1_agent2[0]<<", "<<q1_agent2[1]<<"\n";
                return false;
            }
        }
    }
    return true;
}



QuadAgentsTrajectories QuadMultiRRT::plan(const QuadAgentProblem& problem){
    int n_agents = problem.agents.size();


    QuadAgentsTrajectories paths(n_agents);
    std::vector<std::vector<QuadState>> node_vecs(n_agents); 
    std::vector<std::vector<double>> timing_vecs(n_agents); 

    // initialize with agent's q_init
    for(int k=0; k<n_agents; k++){
        node_vecs[k].push_back(problem.agents[k].q_init);
    }

    //parent node lookup table for each agent
    std::vector<std::unordered_map<uint32_t,uint32_t>> parents(n_agents);


    // create cspaces for every agent:
    int sz0 = 200;
    int sz1 = 200;
    std::vector<myCSpace2d> cspaces;
    for(int k=0;k<n_agents; k++){
        myCSpace2d temp(sz0, sz1, problem.env.x_min, problem.env.x_max, problem.env.y_min, problem.env.y_max);
        temp.constructFromCircleAgent(problem.env, problem.agents[k].radius);
        cspaces.push_back(temp);
        // amp::Visualizer::makeFigure(cspaces[k]);
    }
    bool overall_success = true;
    for(int k=0; k<n_agents; k++){
        //RRT stuff
        QuadState q_sample;
        QuadState q_near;
        QuadState q_candidate,edge_candidate; //point that sampled point gets cut down to 

        std::vector<int> level{0}; //keeps track of how deep a node is in the tree
        bool indi_success=false;
        int loops=0;
        int i=1;
        std::cout<<"Running kinodynamic RRT. k="<< k <<"\n";
        do{
            
            if(amp::RNG::randf()<_p_goal)
                q_sample = problem.agents[k].q_goal;
            else 
                q_sample = QuadAgentTools::sampleSpace(problem.env,problem.agents[k]);
                
            uint32_t idx_near= QuadAgentTools::getNearestNeighbor(node_vecs[k],q_sample);
            q_near = node_vecs[k][idx_near];

            q_candidate = QuadAgentTools::steer(problem.env, problem.agents[k],q_near,q_sample,_Dt);
            
            // flag to keep track of if there is a free path betw points
            bool edge_clear = true;
            Eigen::Vector2d temp1,temp2;
            temp1<<q_near(0),q_near(1);
            temp2<<q_candidate(0),q_candidate(1);
            if(!cspaces[k].freeBtwPoints(temp1, temp2)){
                edge_clear = false;
                // std::cout<<"Edge failed in CSpace check.\n";
                continue;
            }


            // check for collisions with agents who already have a plan
            if(edge_clear && !stepFreeAtTime(problem, paths, q_near, q_candidate, k, level[idx_near]+1)){
                edge_clear = false;
                continue;
            }
            // std::cout<<"Made it past stepFreeAtTime\n";


            if(edge_clear){
                // std::cout<<"adding point to RRT: "<<q_candidate[0]<<", "<<q_candidate[1]<<"\n";
                node_vecs[k].push_back(q_candidate);
                parents[k][i]=idx_near;
                level.push_back(level[parents[k][i]]+1);
                i++; 

                if(QuadAgentTools::distFunc(q_candidate, problem.agents[k].q_goal) < _epsilon){
                    indi_success=true;
                    if(q_candidate!=problem.agents[k].q_goal){
                        node_vecs[k].push_back(problem.agents[k].q_goal);
                        parents[k][i] = i-1;
                    }
                    break;
                }
            }

        }while(loops++ < _N_MAX);

        // update variable that tracks how many nodes 
        // _tree_size+=node_vecs[k].size();

        // need to create the path for current robot
        QuadAgentTrajectory path;
        uint32_t curr = node_vecs[k].size()-1;
        while(parents[k].find(curr) != parents[k].end()){
            path.push_back(node_vecs[k][curr]);
            curr = parents[k][curr];
        }
        path.push_back(node_vecs[k][curr]);
        std::reverse(path.begin(),path.end());
        paths[k] = path;

        if(!indi_success){
            std::cout<<"RRT did not find the goal\n";
            overall_success = false;
            break;
        }

    }
    
    return paths;
}