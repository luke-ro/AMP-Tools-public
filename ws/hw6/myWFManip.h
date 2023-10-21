#include "AMPCore.h"
#include "limits.h"
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"
#include "myWaveFront.h"
#include "MyGridCon.h"      

inline double wrap_0_2pi(double ang){
    double pi = 3.1415;
    while(ang>=2.0*pi){
        ang-=2*pi;
    }
    while(ang<0.0){
        ang+=2.0*pi;
    }
    return ang;
}

class myWFManip: public amp::ManipulatorWaveFrontAlgorithm{
    public:

    myWFManip(const MyGridCon& c_space_constructor)
    :amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCon>(c_space_constructor)), 
    _cell_width(0.25)
    {}

    myWFManip(const MyGridCon& c_space_constructor, double width)
    :amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCon>(c_space_constructor)), 
    _cell_width(width)
    {}

    // myWFManip(double width): _cell_width(width){}
    
    virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override{
        Eigen::Vector2d q_init_wrap {wrap_0_2pi(q_init[0]),wrap_0_2pi(q_init[1])};
        Eigen::Vector2d q_goal_wrap {wrap_0_2pi(q_goal[0]),wrap_0_2pi(q_goal[1])};

        return myWaveFront::planInCSpace(q_init_wrap, q_goal_wrap, grid_cspace, true);
    }

    // virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override{
    //     return myWaveFront::constructDiscretizedWorkspace(environment,_cell_width);
    // }

    private:

    double _cell_width;

};