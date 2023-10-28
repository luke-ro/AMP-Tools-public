#include "myPRM2D.h"

amp::Path2D myPRM2D::plan(const amp::Problem2D& problem){
    // initialize q_init and q_goalS
    amp::ShortestPathProblem spp;
    auto ptr = std::make_shared<amp::Graph<double>>();
    spp.graph = ptr;
    spp.init_node = amp::Node(0);
    spp.goal_node = amp::Node(1);
    
    // for some amount of samples, sample and add if free
    Eigen::Vector2d sample;
    
    std::vector<Eigen::Vector2d> node_locs;
    node_locs.push_back(problem.q_init);
    node_locs.push_back(problem.q_goal);
    for(int i=0; i<_N_MAX; i++){
        sample = H::sampleSpace(problem);
        if(H::checkCollsionEnv(problem,sample)){
            node_locs.push_back(sample);
        }
    }
    

    // connect samples within some distance of eachother
        // check if path is free, then connect
    // add heuristic to nodes
    amp::LookupSearchHeuristic heur;
    
    double dist;
    {int i=0;
    for(auto loc : node_locs){
        heur.heuristic_values[i]=(loc-problem.q_goal).norm();
        std::vector<amp::Node> neighbors = H::getNeighbors(node_locs, loc, _neigh_radius, i);
        for(int j=0; j<neighbors.size(); j++){
            dist = (node_locs[i]-node_locs[j]).norm();
            spp.graph->connect(i,j,dist);
            spp.graph->connect(j,i,dist);
        }
    }i++;}

    // graph search
        // A*
    myAStar star;
    amp::AStar::GraphSearchResult gsr = star.search(spp, heur);
    
    amp::Path2D path;
    for(auto n : gsr.node_path){
        path.waypoints.push_back(node_locs[n]);
    }

    return path;

}