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

    // make a amp::MultiAgentPath2D to store all the paths
    // amp::MultiAgentPath2D paths(problem.numAgents());
    
    // vector of rrts (vector for each agent of nodes) 
    std::vector<Eigen::Matrix<double, 2*n_agents, 1>> noed_vec(n_agents); 

    // vector of parents for each agent
    std::unordered_map<uint32_t,uint32_t> parents(n_agents);


    {int k = 0;
    for(auto tree : trees){
        tree.push_back(problem.agent_properties[k].q_init);
        k++;
    }}

    // the state vector looks like:
    // [[agent1_x, agent1_y],
    //  [agent2_x, agent2_y],
    //  ...
    //  [agentn_x, agentn_y]]

    //set goal and start config
    Eigen::Matrix<double, 2*n_agents, 1> qs_init;
    Eigen::Matrix<double, 2*n_agents, 1> qs_goal;

    for(int k = 0; k<n_agents; k++){
        qs_init(2*k) = problem.agent_properties[k].q_init[0];
        qs_init(2*k+1) = problem.agent_properties[k].q_init[1];

        qs_goal(2*k) = problem.agent_properties[k].q_goal[0];
        qs_goal(2*k+1) = problem.agent_properties[k].q_goal[1];
    }


    //RRT stuff
    Eigen::Matrix<double, 2*n_agents, 1> q_sample;
    Eigen::Matrix<double, 2*n_agents, 1> q_near;
    Eigen::Matrix<double, 2*n_agents, 1> q_candidate;
    Eigen::Matrix<double, 2*n_agents, 1> edge_candidate; //point that sampled point gets cut down to 

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
            // if(edge_candidate.norm()>_radius)
            //     edge_candidate = edge_candidate*_radius/edge_candidate.norm();
            edge2d[0] = edge_candidate(2*k);
            edge2d[1] = edge_candidate(2*k+1);
            if(norm2d(edge2d[0],edge2d[1]) > _radius){
                edge2d = edge2d*_radius/edge2d.norm();
                edge_candidate(2*k) = edge2d[0];
                edge_candidate(2*k+1) = edge2d[1];
            }
        }

        q_candidate = q_near + edge_candidate;
        
        bool edge_clear = true;
        for(int k=0;k<n_agents; k++){
            if(!H::freeBtwPointsGrid(cspace_vec[k],q_near,q_candidate)){
                edge_clear = false;
                break;
            }
        }

        if(H::freeBtwPointsLine(problem,q_near,q_candidate)){
            // std::cout<<"adding point to RRT\n";
            node_vec.push_back(q_candidate);
            parents[i]=idx_near;
            spp.graph->connect(idx_near,i,edge_candidate.norm());
            i++; 

            if((q_candidate-problem.q_goal).norm()<_epsilon){
                success=true;
                if(q_candidate!=problem.q_goal){
                    node_vec.push_back(problem.q_goal);
                    parents[i] = i-1;
                }
                break;
            }
        }

    }while(loops++<_N_MAX);
    
    if(!success){
        std::cout<<"RRT did not find the goal\n";
    }

    amp::Path2D path;
    uint32_t curr = node_vec.size()-1;
    while(parents.find(curr) != parents.end()){
        path.waypoints.push_back(node_vec[curr]);
        curr = parents[curr];
    }
    path.waypoints.push_back(node_vec[curr]);
    std::reverse(path.waypoints.begin(),path.waypoints.end());
    // path.waypoints.push_front(problem.q_init);
    // path.waypoints.insert(path.waypoints.begin(), problem.q_init);

    // Smooothing is probably bad in this context
    // if(_smoothing)
    //     smoothPath(problem,path);

    if(_save_data){
        _node_locs = node_vec;
        _graph_ptr = spp.graph;
    }else{
        _node_locs.clear();
        // if(_graph_ptr->nodes().size()>1) _graph_ptr->clear();
    }
    
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