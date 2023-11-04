#include "myRRT2D.h"

amp::Path2D myRRT2D::plan(const amp::Problem2D& problem){
    // initialize q_init and q_goalS
    amp::ShortestPathProblem spp;
    auto ptr = std::make_shared<amp::Graph<double>>();
    spp.graph = ptr;
    spp.init_node = amp::Node(0);
    spp.goal_node = amp::Node(1);
    
    
    std::vector<Eigen::Vector2d> node_vec(1); 
    node_vec[0] = problem.q_init;

    std::unordered_map<uint32_t,uint32_t> parents;
    

    //RRT stuff
    Eigen::Vector2d q_sample;
    Eigen::Vector2d q_near;
    Eigen::Vector2d q_candidate,edge_candidate; //point that sampled point gets cut down to 

    bool success=false;
    int loops=0;
    int i=1;
    do{
        
        if(amp::RNG::randf()<_p_goal)
            q_sample = problem.q_goal;
        else 
            q_sample = H::sampleSpace(problem);
            
        uint32_t idx_near= H::getNearestNeighbor(node_vec,q_sample);
        q_near = node_vec[idx_near];

        //cut down path to radius _radius
        edge_candidate = q_sample-q_near;
        if(edge_candidate.norm()>_radius)
            edge_candidate = edge_candidate*_radius/edge_candidate.norm();

        q_candidate = q_near + edge_candidate;
        
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

    if(_smoothing)
        smoothPath(problem,path);

    if(_save_data){
        _node_locs = node_vec;
        _graph_ptr = spp.graph;
    }else{
        _node_locs.clear();
        // if(_graph_ptr->nodes().size()>1) _graph_ptr->clear();
    }
    
    return path;

}