#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"

#include "myAStar.h"
#include "Helpers.h"
#include "mySamplingBasedMthd.h"

class myPRM2D : public mySamplingBasedMthd{
    public:



    myPRM2D(int N=1000, double r=1.0, bool smoothing=true, bool save=false):
        mySamplingBasedMthd(N,r,smoothing,save){}


    virtual amp::Path2D plan(const amp::Problem2D& problem) override;


  

    private:



};