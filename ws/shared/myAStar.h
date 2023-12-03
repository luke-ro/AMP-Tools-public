#pragma once

#include "AMPCore.h"

//c++ stuff 
#include "limits.h"
#include <algorithm>

// my stuff
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"

class myAStar : public amp::AStar{
    public:

    virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
    amp::AStar::GraphSearchResult backTrackPath(std::shared_ptr<amp::Graph<double>>  gph, std::unordered_map<amp::Node,amp::Node> parents, amp::Node node);

};