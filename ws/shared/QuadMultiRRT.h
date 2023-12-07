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

    QuadMultiRRT(int N_MAX=10000, double Dt=0.1, double p_goal=0.05, double epsilon=0.25):
        _N_MAX (N_MAX),
        _Dt (Dt),
        _p_goal (p_goal),
        _epsilon (epsilon)
    {}

    QuadAgentsTrajectories plan(const QuadAgentProblem& problem);

    private:

    int _N_MAX;
    double _Dt;
    double _p_goal;
    double _epsilon;
    
};