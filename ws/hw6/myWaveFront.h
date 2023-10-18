#include "AMPCore.h"
#include "limits.h"
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"

class myWaveFront : public amp::PointWaveFrontAlgorithm{
    public:

    myWaveFront():_cell_width(.25){};

    //Functions I am overiding
    virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
    virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;

    private:

    int _sz_x0;
    int _sz_x1;
    Eigen::Vector2d _x0_bounds;
    Eigen::Vector2d _x1_bounds;
    double _cell_width;

    // CSpace2D _cspace;
    // ~myWaveFront(){}
};