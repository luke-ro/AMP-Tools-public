#include "AMPCore.h"
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"

class myWaveFront : public amp::PointWaveFrontAlgorithm{
    public:

    // myWaveFront(){};

    //Functions I am overiding
    virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
    virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;

    // ~myWaveFront(){}
};