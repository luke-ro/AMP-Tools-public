#include "AMPCore.h"

//c++ stuff 
#include "limits.h"
#include <algorithm>

// my stuff
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"
#include "myWaveFront.h"

template<class T>
inline bool contains(const std::list<T>& l, T var){
    return (std::find(l.begin(),l.end(),var) != l.end());
}


inline void priorityAdd(std::list<amp::Node>& arr, std::unordered_map<amp::Node,double> vals, int idx){
    for(std::list<amp::Node>::iterator i=arr.begin(); i!=arr.end(); i++){
        if (vals[idx]<vals[*i]){
            arr.insert(i,idx);
            return;
        }
    }
    arr.push_back(idx);
    return;
}   

class myAStar : public amp::AStar{
    public:

    virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
    amp::AStar::GraphSearchResult backTrackPath(std::unordered_map<amp::Node,amp::Node> parents, amp::Node node);

};

amp::AStar::GraphSearchResult myAStar::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){
    
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
    while(!frontier.empty()){
        curr = frontier.front();
        frontier.pop_front();

        if (curr==problem.goal_node){
            return backTrackPath(parent_of,curr);
        }

        int i=0; //this is needed to match index between edges and children nodes
        for(auto ch : problem.graph->children(curr)){{
            double tentative_gscore = gScore[curr] + problem.graph->outgoingEdges(curr)[i]; //score to curr + score of curr to child

            if(tentative_gscore<gScore[ch]){
                //This path is better than any previous
                parent_of[ch] = curr;
                gScore[ch] = tentative_gscore;
                fScore[ch] = tentative_gscore + heuristic(ch);
                if (!contains(frontier,ch)){
                    priorityAdd(frontier, fScore, ch);
                }
            }
            
        }i++;}

    }
    std::cout<<"AStar completed without reaching goal.";
    return backTrackPath(parent_of, curr);
}

// returns path from node to graph start using the parents array
amp::AStar::GraphSearchResult myAStar::backTrackPath(std::unordered_map<amp::Node,amp::Node> parents, amp::Node node){
    amp::AStar::GraphSearchResult path;
    amp::Node curr = node;
    while(parents.find(curr) != parents.end()){ //if the key does not exist it is the start node (i.e. does not have a parent)
        path.node_path.push_front(curr);
        curr = parents[curr];
    }
    path.node_path.push_front(curr);
    return path;
}