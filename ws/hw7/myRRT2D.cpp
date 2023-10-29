#include "myRRT2D.h"

amp::Path2D myPRM2D::plan(const amp::Problem2D& problem){
    // initialize q_init and q_goalS
    amp::ShortestPathProblem spp;
    auto ptr = std::make_shared<amp::Graph<double>>();
    spp.graph = ptr;
    spp.init_node = amp::Node(0);
    spp.goal_node = amp::Node(1);
    
    
    //RRT stuff
    
    amp::Path2D path;

    if(_smoothing)
        smoothPath(problem,path);

    if(_save_data){
        _node_locs = node_locs;
        _graph_ptr = spp.graph;
    }else{
        _node_locs.clear();
        // if(_graph_ptr->nodes().size()>1) _graph_ptr->clear();
    }
    
    return path;

}