#include "AMPCore.h"
#include "limits.h"
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"
#include "myWaveFront.h"

class myAStar : public amp::AStar{
    public:

    virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;

};

amp::AStar::GraphSearchResult myAStar::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){
    return GraphSearchResult();
}