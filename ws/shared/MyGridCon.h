#pragma once

#include "AMPCore.h"
#include "CSpace2D.h"
#include "hw/HW4.h"

class MyGridCon : public amp::GridCSpace2DConstructor {
    public:
    MyGridCon(int n1=100, int n2 = 100): _n1(n1),_n2(n2){};
    
    std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

    private:

    int _n1;
    int _n2;
};


