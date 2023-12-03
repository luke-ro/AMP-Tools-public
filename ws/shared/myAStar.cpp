#include "myAStar.h"

template<class T>
inline bool contains(const std::list<T>& l, T var){
    return (std::find(l.begin(),l.end(),var) != l.end());
}


inline amp::Node getMin(std::list<amp::Node>& arr, std::unordered_map<amp::Node,double> vals){
    amp::Node min = arr.front();
    for(std::list<amp::Node>::iterator elem=arr.begin(); elem!=arr.end(); elem++){
        if (vals[*elem]<vals[min]){
            min = *elem;
        }
    }

    return min;

}

//TODO need to update priority after each loop!!!
inline void priorityAdd(std::list<amp::Node>& arr, std::unordered_map<amp::Node,double> vals, int idx){
    auto func = [vals](const amp::Node& first, const amp::Node& second)
            { return vals.at(first)<vals.at(second);};
    // for(std::list<amp::Node>::iterator i=arr.begin(); i!=arr.end(); i++){
    //     if (vals[idx]<vals[*i]){
    //         arr.insert(--i,idx);
    //         arr.sort(func);
    //         // std::cout << idx <<" inserted at pos " << *i-1<<"\n";
    //         for(std::list<amp::Node>::iterator j=arr.begin(); j!=arr.end(); j++) 
    //             std::cout<< *j <<"("<<vals[*j]<<"), ";
    //         std::cout<<"\n";
    //         return;
    //     }
    // }
    arr.push_back(idx);
    // std::cout << idx <<" inserted at pos " << arr.size()-1 <<"(end) \n";
    // for(std::list<amp::Node>::iterator j=arr.begin(); j!=arr.end(); j++) 
    //             std::cout<< *j <<", ";
    // std::cout<<"\n";


    arr.sort(func);
    // for(std::list<amp::Node>::iterator j=arr.begin(); j!=arr.end(); j++) 
    //     std::cout<< *j <<"("<<vals[*j]<<"), ";
    // std::cout<<"\n";


    return;
}   


inline amp::AStar::GraphSearchResult myAStar::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){
    
    std::list<amp::Node> frontier = {problem.init_node};

    std::unordered_map<amp::Node,amp::Node> parent_of; //parent_of[n] gives the amp::nodes that is the parent    
    std::unordered_map<amp::Node,double> gScore; 
    std::unordered_map<amp::Node,double> fScore; 

    for(auto n : problem.graph->nodes()){
        gScore[n] = std::numeric_limits<double>::max();
        gScore[n] = std::numeric_limits<double>::max();
    }

    gScore[problem.init_node] = 0;
    fScore[problem.init_node] = heuristic(problem.init_node);
    // parent_of[problem.init_node] = NULL;

    amp::Node curr;
    int k = 0;
    while(!frontier.empty()){
        curr = getMin(frontier,fScore);
        frontier.remove(curr);

        if (curr==problem.goal_node){
            amp::AStar::GraphSearchResult to_ret = backTrackPath(problem.graph,parent_of, curr);
            to_ret.success = true;
            to_ret.path_cost = gScore[curr];
            // for(auto i : gScore) std::cout<<"gScore "<<i.first<<": " <<i.second<<"\n";
            // for(auto i : fScore) std::cout<<"fScore "<<i.first<<": " <<i.second<<"\n";
            // for(auto i : parent_of) std::cout<<"parent "<<i.first<<": " <<i.second<<"\n";
            std::cout<<"goal found in "<<k<< "iterations"<<"\n";
            return to_ret;
        }

        int i=0; //this is needed to match index between edges and children nodes
        // std::cout<<"looking at children of "<< curr <<". Chidren: ";
        for(auto ch : problem.graph->children(curr)){{
            double tentative_gscore = gScore[curr] + problem.graph->outgoingEdges(curr)[i]; //score to curr + score of curr to child
            // std::cout<<ch<<" ("<<tentative_gscore<<"), ";
            if(tentative_gscore<gScore[ch]){
                //This path is better than any previous
                parent_of[ch] = curr;
                gScore[ch] = tentative_gscore;
                fScore[ch] = tentative_gscore + heuristic(ch);
            
                if (!contains(frontier,ch)){
                        // priorityAdd(frontier, fScore, ch);
                        frontier.push_back(ch);
                }
            }
            
        }i++;}
        
        // std::cout<<"\n";
    k++;
    }

    if(parent_of.find(problem.goal_node)!=parent_of.end()){
            amp::AStar::GraphSearchResult to_ret = backTrackPath(problem.graph,parent_of, problem.goal_node);
            to_ret.success = true;
            to_ret.path_cost = gScore[problem.goal_node];
            std::cout<<"goal found in "<<k<< "iterations"<<"\n";
            return to_ret;
    }

    std::cout<<"AStar completed without reaching goal.";
    amp::AStar::GraphSearchResult to_ret;
    to_ret.node_path.push_back(problem.init_node);
    return to_ret;
}

// returns path from node to graph start using the parents array
amp::AStar::GraphSearchResult myAStar::backTrackPath(std::shared_ptr<amp::Graph<double>> gph, std::unordered_map<amp::Node,amp::Node> parents, amp::Node node){
    amp::AStar::GraphSearchResult path;
    amp::Node curr = node;
    double cost=0;
    do{ 
        path.node_path.push_front(curr); //add to path

        amp::Node parent = parents[curr]; // get parent of current node 
        // std::vector<amp::Node> children = gph->children(parent); //get children of parent
        // std::vector<double> outgoing_edges = gph->outgoingEdges(parent); //get outgoing edges of parent
        
        // //loop through children
        // for(int i=0;i<children.size();i++){

        //     // if current child is the curr node, add cost of edge
        //     if(children[i]==curr){
        //         cost += outgoing_edges[i];
        //         break;
        //     }
        //     std::cout<<"a child was not found"<<"\n";
        // }
        
        // go to next higher node (parent)
        curr = parent;
    }while(parents.find(curr) != parents.end()); //stop when at end of list

    path.node_path.push_front(curr);
    // std::reverse(path.node_path.begin(),path.node_path.end());
    path.path_cost = cost;
    return path;
}