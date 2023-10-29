#include "myPRM2D.h"

void smoothPath(const amp::Problem2D& prob, amp::Path2D& path){
    double dist;
    int len_start = path.waypoints.size();
    if(len_start>2){
        int i,j;
        for(int k=0; k<len_start*2; k++){
            int len = path.waypoints.size();
            // std::cout<<"one iteration\n";
            i = amp::RNG::randi(0,len);
            j = amp::RNG::randi(0,len);
            if(abs(j-i)<=1)
                continue;

            Eigen::Vector2d p1 = path.waypoints[i];
            Eigen::Vector2d p2 = path.waypoints[j];
            dist = (p1-p2).norm();
            if(H::freeBtwPoints(prob, p1, p2, int(dist*10.0))){
                //remove nodes between i and j
                if(j<i){
                    int temp = i;
                    i = j;
                    j = temp;
                }
                // std::cout<<"Erasing from ["<<i<<", "<<j<<") len: "<<path.waypoints.size()<<"\n";
                path.waypoints.erase(path.waypoints.begin()+i+1,path.waypoints.begin()+j);                  ;
            }
        }
    }
    return;
}

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
        if(!H::checkCollsionEnv(problem,sample)){
            node_locs.push_back(sample);
        }
    }
    
    // connect samples within some distance of eachother
        // check if path is free, then connect
    // add heuristic to nodes
    amp::LookupSearchHeuristic heur;
    
    double dist;
    int i=0;
    for(auto loc : node_locs){{
        heur.heuristic_values[i]=(loc-problem.q_goal).norm();
        std::vector<amp::Node> neighbors = H::getNeighbors(node_locs, loc, _neigh_radius, i);
        for(int j=0; j<neighbors.size(); j++){
            dist = (node_locs[i]-node_locs[neighbors[j]]).norm();
            bool fbp = H::freeBtwPoints(problem, node_locs[i], node_locs[neighbors[j]], int(dist*20.0));
            if(H::freeBtwPoints(problem, node_locs[i], node_locs[neighbors[j]], int(dist*20.0))){
                spp.graph->connect(neighbors[j],i,dist);
                spp.graph->connect(i,neighbors[j],dist);
            }
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



void myPRM2D::getData(std::shared_ptr<amp::Graph<double>>& g_ptr, std::map<amp::Node, Eigen::Vector2d>& m){
    if(!_save_data){
        std::cout<<"getPRMData() called but _save_data set to false.\n";
        g_ptr = _graph_ptr; 
        m[0] = Eigen::Vector2d();
        return;
    }
    
    g_ptr = _graph_ptr; 

    {int i=0;
    for(auto p : _node_locs){
        m[i] = p;
    i++;}}
    return;
}