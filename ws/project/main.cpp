#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <iostream>

#include "AMPCore.h"
#include "hw/HW4.h"
#include "Helpers.h"

namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Agent{
    double radius = 1.0;
    double x_start = 0;
    double y_start = 0;
    double theta_start = 0;

    double x_goal = 1;
    double y_goal = 1;
    double theta_goal = 1;
};

class World{
    public:

    amp::Problem2D problem;
    std::vector<std::shared_ptr<Agent>> agents;

};

class isStateValid : public ob::StateValidityChecker{
    public:
    isStateValid(const ob::SpaceInformationPtr &si, const std::shared_ptr<World> w, const std::shared_ptr<Agent> a):
        ob::StateValidityChecker(si),
        _w(w),
        _a(a){

        _si = si.get();
    }

    bool isValid(const ob::State *state) const override{
        if(!_si->satisfiesBounds(state))
            return false;
        
        // TODO
        return true;
    }

    private:

    const ob::SpaceInformation *_si;
    const std::shared_ptr<World> _w;
    const std::shared_ptr<Agent> _a;

};

og::SimpleSetupPtr geoSetup(const std::shared_ptr<World> w){
    // create pointer to the ss work space? 
    auto ws(std::make_shared<ob::SE2StateSpace>());

    std::shared_ptr<Agent> a = w->agents[0];

    //pointer to bouds object with 2 dimensions
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, w->problem.x_min);
    bounds.setHigh(0, w->problem.x_max);    
    bounds.setLow(1, w->problem.y_min);
    bounds.setHigh(1, w->problem.y_max);

    ws->setBounds(bounds);

    auto ss = std::make_shared<og::SimpleSetup>(ws);

    ss->setStateValidityChecker(std::make_shared<isStateValid>(ss->getSpaceInformation(),w,a));


    // setup agent
    ob::ScopedState<ob::SE2StateSpace> start(ws);
    start->setX(a->x_start);
    start->setY(a->y_start);
    start->setYaw(a->theta_start);

    ob::ScopedState<ob::SE2StateSpace> goal(ws);
    goal->setX(a->x_goal);
    goal->setY(a->y_goal);

    ss->setStartAndGoalStates(start,goal,0.1);
    OMPL_INFORM("Setup problem");
    return ss;
}

void planGeometric(const std::shared_ptr<World> w){
    

    // create simple setup pointer
    og::SimpleSetupPtr ss =  geoSetup(w);

    // create planner object
    ob::PlannerPtr planner = std::make_shared<og::RRT>(ss->getSpaceInformation());
    ss->setPlanner(planner);

    bool solved = ss->solve(30.0);
    if(solved){
        ss->simplifySolution();
        OMPL_INFORM("FOUND SOLUTION");
    }
}

int main(int argc, char ** argv)
{   

    auto agent = std::make_shared<Agent>();

    std::vector<std::shared_ptr<Agent>> test;
    test.push_back(agent);

    std::shared_ptr<World> w;
    w->agents.push_back(agent);

    amp::Environment2D ws = amp::HW4::getEx3Workspace1();
    w->problem.obstacles = ws.obstacles;
    w->problem.x_max = ws.x_max;
    w->problem.x_min = ws.x_min;
    w->problem.y_max = ws.y_max;
    w->problem.y_min = ws.y_min;
    w->problem.q_init[0] = 0;
    w->problem.q_init[1] = 0;
    w->problem.q_goal[0] = 4;
    w->problem.q_goal[1] = 4;

    // std::string plannerName = "RRT";
    OMPL_INFORM("Planning for OMPL Lecture Example using Gemoetric Planning with %s", "RRT");
    planGeometric(w);
}