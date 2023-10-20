#pragma once

#include "AMPCore.h"
#include "limits.h"
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"

class myWaveFront{
    public:

    myWaveFront():_cell_width(.25){};

    //Functions I am overiding
    static amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool wrap=false);
    static std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment, double cell_width);

    private:

    int _sz_x0;
    int _sz_x1;
    Eigen::Vector2d _x0_bounds;
    Eigen::Vector2d _x1_bounds;
    double _cell_width;

    // CSpace2D _cspace;
    // ~myWaveFront(){}
};