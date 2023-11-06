#include "myDecenMultiRRT.h"

bool stepFreeAtTime(const amp::MultiAgentProblem2D& prob, const std::vector<amp::Path2D>& paths, Eigen::Vector2d q1,Eigen::Vector2d q2, int idx_agent, int idx_time){
    // the agent being checked
    Eigen::Vector2d q1_agent2 = q1;
    Eigen::Vector2d q2_agent2 = q2;

    //loop through agents that already have plans
    for(int k=0;k<idx_agent; k++){
        int n = 10;
        double min_r = 1.0*(prob.agent_properties[idx_agent].radius + prob.agent_properties[k].radius);

        //points for higher priority agent (agents with plans)
        Eigen::Vector2d q1_agent1;
        Eigen::Vector2d q2_agent1;
        
        // check to make sure that the agent was moving at that time. 
        int sz_agent1 = paths[k].waypoints.size();
        if(sz_agent1-1<idx_time){
            q1_agent1 = paths[k].waypoints[sz_agent1-1];
            q1_agent2 = paths[k].waypoints[sz_agent1-1];
        }else{
            q1_agent1 = paths[k].waypoints[idx_time-1];
            q2_agent1 = paths[k].waypoints[idx_time];
        }


        std::vector<Eigen::Vector2d> path_agent1  = H::linspace2D(q1_agent1,q2_agent1,n);
        std::vector<Eigen::Vector2d> path_agent2  = H::linspace2D(q1_agent2,q2_agent2,n);

        for(int i = 0; i<n; i++){
            if((path_agent1[i]-path_agent2[i]).norm() < min_r){
                return false;
            }
        }
    }
    return true;
}

amp::MultiAgentPath2D myDecenMultiRRT::plan(const amp::MultiAgentProblem2D& problem){
    int n_agents = problem.agent_properties.size();

    std::vector<amp::Path2D> paths(n_agents);
    std::vector<std::vector<Eigen::Vector2d>> node_vecs(n_agents); 
    std::vector<std::vector<double>> timing_vecs(n_agents); 

    // initialize with agent's q_init
    for(int k=0; k<n_agents; k++){
        node_vecs[k].push_back(problem.agent_properties[k].q_init);
    }

    //parent node lookup table for each agent
    std::vector<std::unordered_map<uint32_t,uint32_t>> parents(n_agents);

    // create cspaces for every agent:
    int sz0 = 200;
    int sz1 = 200;
    std::vector<myCSpace2d> cspaces;
    for(int k=0;k<n_agents; k++){
        myCSpace2d temp(sz0, sz1, problem.x_min, problem.x_max, problem.y_min, problem.y_max);
        temp.constructFromCircleAgent(problem, problem.agent_properties[k]);
        cspaces.push_back(temp);
        // amp::Visualizer::makeFigure(cspaces[k]);
    }
    
    bool overall_success = true;
    for(int k=0; k<n_agents; k++){
        //RRT stuff
        Eigen::Vector2d q_sample;
        Eigen::Vector2d q_near;
        Eigen::Vector2d q_candidate,edge_candidate; //point that sampled point gets cut down to 

        bool indi_success=false;
        int loops=0;
        int i=1;
        std::cout<<"Running RRT. k="<< k <<"\n";
        do{
            
            if(amp::RNG::randf()<_p_goal)
                q_sample = problem.agent_properties[k].q_goal;
            else 
                q_sample = H::sampleSpace(problem);
                
            uint32_t idx_near= H::getNearestNeighbor(node_vecs[k],q_sample);
            q_near = node_vecs[k][idx_near];

            //cut down path to radius _radius
            edge_candidate = q_sample-q_near;
            if(edge_candidate.norm()>_radius)
                edge_candidate = edge_candidate*_radius/edge_candidate.norm();

            q_candidate = q_near + edge_candidate;
            
            // flag to keep track of if there is a free path betw points
            bool edge_clear = true;

            if(!cspaces[k].freeBtwPoints(q_near,q_candidate)){
                edge_clear = false;
                // std::cout<<"Edge failed in CSpace check.\n";
                continue;
            }


            // check for collisions with agents who already have a plan
            if(edge_clear && !stepFreeAtTime(problem, paths, q_near, q_candidate, k, i+1)){
                edge_clear = false;
                continue;
            }
            // std::cout<<"Made it past stepFreeAtTime\n";


            if(edge_clear){
                std::cout<<"adding point to RRT\n";
                node_vecs[k].push_back(q_candidate);
                parents[k][i]=idx_near;
                i++; 

                if((q_candidate-problem.agent_properties[k].q_goal).norm()<_epsilon){
                    indi_success=true;
                    if(q_candidate!=problem.agent_properties[k].q_goal){
                        node_vecs[k].push_back(problem.agent_properties[k].q_goal);
                        parents[k][i] = i-1;
                    }
                    break;
                }
            }

        }while(loops++<_N_MAX);
        if(!indi_success){
            std::cout<<"RRT did not find the goal\n";
            overall_success = false;
            break;
        }

        // need to create the path for each robot
        amp::Path2D path;
        uint32_t curr = node_vecs[k].size()-1;
        while(parents[k].find(curr) != parents[k].end()){
            path.waypoints.push_back(node_vecs[k][curr]);
            curr = parents[k][curr];
        }
        path.waypoints.push_back(node_vecs[k][curr]);
        std::reverse(path.waypoints.begin(),path.waypoints.end());
        paths[k] = path;
    }
    

    amp::MultiAgentPath2D agent_paths(n_agents);
    agent_paths.valid = overall_success;
    agent_paths.agent_paths = paths;

    // path.waypoints.push_front(problem.q_init);
    // path.waypoints.insert(path.waypoints.begin(), problem.q_init);

    // if(_smoothing)
    //     smoothPath(problem,path);

    // if(_save_data){
    //     _node_locs = node_vec;
    //     _graph_ptr = spp.graph;
    // }else{
    //     _node_locs.clear();
    //     // if(_graph_ptr->nodes().size()>1) _graph_ptr->clear();
    // }
    
    return agent_paths;
}