#include "AMPCore.h"
#include "limits.h"
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"
#include "myWaveFront.h"

class myWFPoint : public amp::PointWaveFrontAlgorithm{
    public:

    myWFPoint(): _cell_width(0.25){}
    myWFPoint(double width): _cell_width(width){}
    
    virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override{
        return myWaveFront::planInCSpace(q_init,q_goal,grid_cspace);
    }

    virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override{
        return myWaveFront::constructDiscretizedWorkspace(environment,_cell_width);
    }

    private:

    double _cell_width;

};