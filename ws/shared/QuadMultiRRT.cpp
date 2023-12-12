#include "QuadMultiRRT.h"


bool stepFreeAtTime(const QuadAgentProblem& prob, const QuadAgentsTrajectories& paths, const QuadState& q1, const QuadState& q2, int idx_agent, int idx_time){
    // the agent being checked
    Eigen::Vector2d q1_agent2 = QuadAgentTools::getPos(q1);
    Eigen::Vector2d q2_agent2 = QuadAgentTools::getPos(q2);

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
            q1_agent1 = QuadAgentTools::getPos(paths[k][sz_agent1-1]);
            q2_agent1 = QuadAgentTools::getPos(paths[k][sz_agent1-1]);
        }else{
            q1_agent1 = QuadAgentTools::getPos(paths[k][idx_time-1]);
            q2_agent1 = QuadAgentTools::getPos(paths[k][idx_time]);
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



QuadProblemResult QuadMultiRRT::plan(const QuadAgentProblem& problem){
    int n_agents = problem.agents.size();

    //init trees for plotting

    std::vector<std::map<uint32_t,Eigen::Vector2d>> node_maps(n_agents);
    std::vector<amp::ShortestPathProblem> spprobs(n_agents);
    for(int k=0; k<n_agents; k++){
        auto ptr = std::make_shared<amp::Graph<double>>();
        spprobs[k].graph = ptr;
        // spprobs[k].init_node = QuadAgentTools::getPos(problem.agents[k].q_init);
        // spprobs[k].goal_node = QuadAgentTools::getPos(problem.agents[k].q_goal);

        node_maps[k][0] = QuadAgentTools::getPos(problem.agents[k].q_init);
    }


    QuadAgentsTrajectories paths(n_agents);
    std::vector<std::vector<QuadControl>> controls(n_agents);
    std::vector<std::vector<QuadState>> node_vecs(n_agents); 
    std::vector<std::vector<Eigen::Vector2d>> control_vecs(n_agents); 
    std::vector<std::vector<double>> timing_vecs(n_agents); 

    // initialize with agent's q_init
    for(int k=0; k<n_agents; k++){
        node_vecs[k].push_back(problem.agents[k].q_init);
        control_vecs[k].push_back(Eigen::Vector2d());
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
            // std::cout<<"top of loop"<<"\n";
            
            if(amp::RNG::randf()<_p_goal)
                q_sample = problem.agents[k].q_goal;
            else 
                q_sample = QuadAgentTools::sampleSpace(problem.env,problem.agents[k]);

                
            uint32_t idx_near= QuadAgentTools::getNearestNeighbor(node_vecs[k],q_sample);
            q_near = node_vecs[k][idx_near];
            // std::cout<< "idx_near: " << idx_near << "\n";
            
            if(QuadAgentTools::distFunc(q_near,q_sample)>_radius){
                Eigen::Vector2d temp = QuadAgentTools::getPos(q_sample-q_near);
                temp = temp*_radius/temp.norm();
                q_sample[0] = q_near[0]+temp[0];
                q_sample[1] = q_near[1]+temp[1];
            }

            Eigen::Vector2d control;
            q_candidate = QuadAgentTools::steer(problem.env, problem.agents[k],q_near,q_sample,_Dt,10,control);
            
            // std::cout<<control[0]<<", "<<control[1]<<", ";
            // std::cout<<"q_near: "; QuadAgentTools::printState(q_near);
            // std::cout<<"q_samp: "; QuadAgentTools::printState(q_sample);
            // std::cout<<"q_cand: "; QuadAgentTools::printState(q_candidate);

            if(!QuadAgentTools::withinBounds(problem.env, problem.agents[k], q_candidate))
                continue;

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
                control_vecs[k].push_back(control);
                parents[k][i]=idx_near;
                level.push_back(level[parents[k][i]]+1);

                spprobs[k].graph->connect(idx_near,i, QuadAgentTools::distFunc(q_near,q_candidate));
                node_maps[k][i] = QuadAgentTools::getPos(q_candidate);
                
                i++; 

                if(QuadAgentTools::distFunc(q_candidate, problem.agents[k].q_goal) < _epsilon){
                    indi_success=true;
                    std::cout<<"FOUND GOAL\n";
                    // if(q_candidate!=problem.agents[k].q_goal){
                    //     node_vecs[k].push_back(problem.agents[k].q_goal);
                    //     parents[k][i] = i-1;
                    // }
                    break;
                }
            }

        }while(loops++ < _N_MAX);


        
        // update variable that tracks how many nodes 
        // _tree_size+=node_vecs[k].size();

        // need to create the path for current robot
        QuadAgentTrajectory path;
        std::vector<QuadControl> control;
        uint32_t curr = node_vecs[k].size()-1;
        while(parents[k].find(curr) != parents[k].end()){
            path.push_back(node_vecs[k][curr]);
            control.push_back(control_vecs[k][curr]);
            curr = parents[k][curr];
        }
        path.push_back(node_vecs[k][curr]);
        std::reverse(path.begin(),path.end());
        paths[k] = path;
        controls[k] = control;

        if(!indi_success){
            std::cout<<"RRT did not find the goal\n";
            overall_success = false;
            break;
        }

    }

    _node_maps = node_maps;
    _spprobs = spprobs;

    _controls = controls;

    QuadProblemResult res;
    res.paths = paths;
    res.success = overall_success;
    
    return res;
}