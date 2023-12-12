#pragma once

#include"AMPCore.h"
#include "hw/HW8.h"

#include <random>

#include "myCSpace2d.h"
#include "Helpers.h"
#include "mySamplingBasedMthd.h"
#include "QuadAgentProperties.h"
#include "QuadAgentTools.h"

class QuadMultiRRT{
    public: 

    QuadMultiRRT(int N_MAX=10000, double Dt=0.2, double p_goal=0.05, double epsilon=0.25, double radius=0.5):
        _N_MAX (N_MAX),
        _Dt (Dt),
        _p_goal (p_goal),
        _epsilon (epsilon),
        _radius (radius)
    {}

    QuadProblemResult plan(const QuadAgentProblem& problem);

    std::vector<amp::ShortestPathProblem> getSPPS(){return _spprobs;}
    std::vector<std::map<uint32_t,Eigen::Vector2d>> getNodeMaps(){return _node_maps;}
    std::vector<std::vector<Eigen::Vector2d>> getControlInputs(){return _controls;}

    private:

    int _N_MAX;
    double _Dt;
    double _p_goal;
    double _epsilon;
    double _radius;

    std::vector<amp::ShortestPathProblem> _spprobs;
    std::vector<std::map<uint32_t,Eigen::Vector2d>> _node_maps;
    std::vector<std::vector<Eigen::Vector2d>> _controls;
    
};