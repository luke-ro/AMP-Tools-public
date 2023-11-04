#pragma once

#include"AMPCore.h"
#include "hw/HW8.h"

#include "Helpers.h"
#include "mySamplingBasedMthd.h"

class myCentMultiRRT : public mySamplingBasedMthd, public amp::MultiAgentCircleMotionPlanner2D{
    public: 

    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;



    private:


};