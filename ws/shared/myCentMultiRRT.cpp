#include "myCentMultiRRT.h"

double norm2d(double x1, double x2){
    return sqrt((x1*x1)+(x2*x2));
}


amp::MultiAgentPath2D myCentMultiRRT::plan(const amp::MultiAgentProblem2D& problem){
    // MultiAgentProblem2D-> std::vector<CircularAgentProperties> agent_properties
    //                    -> inline std::size_t numAgents() const {return agent_properties.size();}      
    int n_agents = problem.numAgents();

    // initialize q_init and q_goalS
    amp::ShortestPathProblem spp;
    auto ptr = std::make_shared<amp::Graph<double>>();
    spp.graph = ptr;
    spp.init_node = amp::Node(0);
    spp.goal_node = amp::Node(1);

    
    // vector of rrts (vector for each agent of nodes) 
    std::vector<Eigen::VectorXd> node_vec; 

    // vector of parents for each agent
    std::unordered_map<uint32_t,uint32_t> parents(n_agents);

    //cspaces for every agent:
    int sz0 = 200;
    int sz1 = 200;
    std::vector<myCSpace2d> cspaces;
    for(int k=0;k<n_agents; k++){
        myCSpace2d temp(sz0, sz1, problem.x_min, problem.x_max, problem.y_min, problem.y_max);
        temp.constructFromCircleAgent(problem, problem.agent_properties[k].radius);
        cspaces.push_back(temp);
        // amp::Visualizer::makeFigure(cspaces[k]);
    }



    // the state vector looks like:
    // [agent1_x, agent1_y, agent2_x, agent2_y, ... , agentn_x, agentn_y]

    //set goal and start config
    Eigen::VectorXd q_init(2*n_agents);
    Eigen::VectorXd q_goal(2*n_agents);

    for(int k = 0; k<n_agents; k++){
        q_init(2*k) = problem.agent_properties[k].q_init[0];
        q_init(2*k+1) = problem.agent_properties[k].q_init[1];

        q_goal(2*k) = problem.agent_properties[k].q_goal[0];
        q_goal(2*k+1) = problem.agent_properties[k].q_goal[1];
    }

    // push start config into node vec
    node_vec.push_back(q_init);


    //RRT stuff
    Eigen::VectorXd q_sample(2*n_agents);
    Eigen::VectorXd q_near(2*n_agents);
    Eigen::VectorXd q_candidate(2*n_agents);
    Eigen::VectorXd edge_candidate(2*n_agents); //point that sampled point gets cut down to 

    bool success=false;
    int loops=0;
    int i=1;
    do{
        
        if(amp::RNG::randf()<_p_goal)
            q_sample = q_goal;
        else{
            Eigen::Vector2d sample_2d;
            for(int k=0;k<n_agents; k++){
                sample_2d = H::sampleSpace(problem);
                q_sample(2*k) = sample_2d[0];
                q_sample(2*k+1) = sample_2d[1];
            }
        }
            // qs_sample = H::sampleSpace(problem);
            
        uint32_t idx_near = H::getNearestNeighborNd(node_vec,q_sample);
        q_near = node_vec[idx_near];

        //cut down path to radius _radius
        edge_candidate = q_sample-q_near;
        Eigen::Vector2d edge2d;
        for(int k=0;k<n_agents; k++){
            edge2d[0] = edge_candidate(2*k);
            edge2d[1] = edge_candidate(2*k+1);
            if(edge2d.norm() > _radius){
                edge2d = edge2d*_radius/edge2d.norm();
                edge_candidate(2*k) = edge2d[0];
                edge_candidate(2*k+1) = edge2d[1];
            }
        }

        q_candidate = q_near + edge_candidate;
        
        // flag to keep track of if there is a free path betw points
        bool edge_clear = true;


        //check the cspaces for clear path
        Eigen::Vector2d q_near_2d;
        Eigen::Vector2d q_candidate_2d; 
        for(int k=0;k<n_agents; k++){
            q_near_2d[0] = q_near[2*k];
            q_near_2d[1] = q_near[2*k+1];
            q_candidate_2d[0] = q_candidate[2*k];
            q_candidate_2d[1] = q_candidate[2*k+1];

            if(!cspaces[k].freeBtwPoints(q_near_2d,q_candidate_2d)){
                edge_clear = false;
                // std::cout<<"Edge failed in CSpace check.\n";
                break;
            }
        }

        //check that paths are clear from other agents
        if(edge_clear){ // if the edge is not clear, this loop is moot
            for(int k=0;k<n_agents-1; k++){
                for(int m=k+1; m<n_agents; m++){
                    if(!agentStepsFree(problem, k, m, q_near,q_candidate)){
                        // std::cout<<"Collision detected.\n";
                        edge_clear = false;
                        // std::cout<<"Edge failed in WSpace bot check.\n";
                        break;
                    }
                }
            }
        }

        // add node if all edges are clear and agents don't run into eachother
        if(edge_clear){
            // std::cout<<"adding point to RRT\n";
            node_vec.push_back(q_candidate);
            parents[i]=idx_near;
            // spp.graph->connect(idx_near,i,edge_candidate.norm());
            i++; 
            
            //check if every agent is wihtin epsilon
            bool within_epsilon = true;
            Eigen::Vector2d temp_pt;
            for(int k=0;k<n_agents; k++){
                // if(edge_candidate.norm()>_radius)
                //     edge_candidate = edge_candidate*_radius/edge_candidate.norm();
                temp_pt[0] = q_candidate(2*k)-q_goal(2*k);
                temp_pt[1] = q_candidate(2*k+1)-q_goal(2*k+1);
                if(temp_pt.norm() > _epsilon){
                    within_epsilon = false;
                    break;
                }
            }

            if(within_epsilon){
                success=true;
                if(q_candidate!=q_goal){
                    node_vec.push_back(q_goal);
                    parents[i] = i-1;
                }
                break;
            }
        }

    }while(loops++<_N_MAX);
    
    if(!success){
        std::cout<<"RRT did not find the goal\n";
    }


    // make a amp::MultiAgentPath2D to store all the paths
    amp::MultiAgentPath2D paths(n_agents);
    paths.valid = success;

    // Add paths to return var
    uint32_t curr = node_vec.size()-1; //last node will be the sucessful node
    Eigen::Vector2d pt;
    while(parents.find(curr) != parents.end()){
        for(int k=0; k<n_agents; k++){
            pt[0] = node_vec[curr][2*k];
            pt[1] = node_vec[curr][2*k+1];
            paths.agent_paths[k].waypoints.push_back(pt);
        }
        // path.waypoints.push_back(node_vec[curr]);
        curr = parents[curr];
    }

    // need the following for adding the start node to paths
    for(int k=0; k<n_agents; k++){
        pt[0] = node_vec[curr][2*k];
        pt[1] = node_vec[curr][2*k+1];
        paths.agent_paths[k].waypoints.push_back(pt);
    }

    //reverse the paths so that they go from start to end
    for(int k=0; k<n_agents; k++){
        pt[0] = node_vec[curr][2*k];
        pt[1] = node_vec[curr][2*k+1];
        std::reverse(paths.agent_paths[k].waypoints.begin(), paths.agent_paths[k].waypoints.end());
    }
    

    // Smooothing is probably bad in this context
    // if(_smoothing)
    //     smoothPath(problem,path);

    // if(_save_data){
    //     _node_locs = node_vec;
    //     _graph_ptr = spp.graph;
    // }else{
    //     _node_locs.clear();
    //     // if(_graph_ptr->nodes().size()>1) _graph_ptr->clear();
    // }
    std::cout<<"RRT Tree Size: " << node_vec.size()<<"\n";
    _tree_size = node_vec.size();
    return paths;
}



bool myCentMultiRRT::agentStepsFree(const amp::MultiAgentProblem2D& problem, int idx1, int idx2, Eigen::VectorXd q1_nd, Eigen::VectorXd q2_nd){
    int n = 10;
    double min_r = 1.0*(problem.agent_properties[idx1].radius + problem.agent_properties[idx2].radius);
    
    Eigen::Vector2d q1_agent1 {q1_nd(2*idx1),q1_nd(2*idx1+1)};
    Eigen::Vector2d q2_agent1 {q2_nd(2*idx1),q2_nd(2*idx1+1)};
    Eigen::Vector2d q1_agent2 {q1_nd(2*idx2),q1_nd(2*idx2+1)};
    Eigen::Vector2d q2_agent2 {q2_nd(2*idx2),q2_nd(2*idx2+1)};

    double step_size = std::max((q2_agent1-q1_agent1).norm(), (q2_agent2-q1_agent2).norm());


    std::vector<Eigen::Vector2d> path_agent1  = H::linspace2D(q1_agent1,q2_agent1,n);
    std::vector<Eigen::Vector2d> path_agent2  = H::linspace2D(q1_agent2,q2_agent2,n);

    for(int i = 0; i<n; i++){
        if((path_agent1[i]-path_agent2[i]).norm() < min_r){
            return false;
        }
    }

    return true;

}

/*
Must return a MultiAgentPath2D

    struct MultiAgentPath2D {
        MultiAgentPath2D() = default;
        MultiAgentPath2D(uint32_t n_agents) : agent_paths(n_agents) {}
        std::vector<Path2D> agent_paths;

        /// @brief `true` if a solution was found, `false` otherwise
        bool valid;

        inline std::size_t numAgents() const {return agent_paths.size();}
    };

*/