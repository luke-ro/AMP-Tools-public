#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"

#include "Helpers.h"

class myPRM2D : public amp::PRM2D{
    public:

    myPRM2D(int N=1000):_N_MAX(N){}

    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

    private:

    const int _N_MAX;
};