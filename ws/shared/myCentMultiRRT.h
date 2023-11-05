#pragma once

#include"AMPCore.h"
#include "hw/HW8.h"

#include "myCSpace2d.h"
#include "Helpers.h"
#include "mySamplingBasedMthd.h"

class myCentMultiRRT : public mySamplingBasedMthd, public amp::MultiAgentCircleMotionPlanner2D{
    public: 

    myCentMultiRRT(int N=1000, double r=1.0, double p_goal=0.05, double epsilon=0.25, bool smoothing=true, bool save=false):
        mySamplingBasedMthd(N, r, smoothing, save),
        _p_goal (p_goal),
        _epsilon (epsilon)
    {}

    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

    private:

    double _p_goal;
    double _epsilon;
};