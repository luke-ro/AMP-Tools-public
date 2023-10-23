#include "AMPCore.h"
#include "limits.h"
#include "Helpers.h"
#include "hw/HW6.h"
#include "CSpace2D.h"
#include "myWaveFront.h"
#include "MyGridCon.h"      
#include "hw/HW4.h"
#include "Arm2L.h"

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
        std::cout<<"planning in cpsace with q_init: "<<q_init_wrap[0]<<", "<<q_init_wrap[1]<<", q_goal: "<<q_goal_wrap[0]<<", "<<q_goal_wrap[1]<<"\n";

        std::vector<double> lengths {1.0,1.0};
        Arm2L manip(lengths);

        Eigen::Vector2d temp =  manip.getJointLocation(q_init_wrap,2);
        Eigen::Vector2d temp2 =  manip.getJointLocation(q_goal_wrap,2);
        Eigen::Vector2d temp_state = manip.getConfigurationFromIK(Eigen::Vector2d(0.3,-.3));
        Eigen::Vector2d temp3 =  manip.getJointLocation(temp_state,2);


        std::cout<<"q_init: "<<q_init[0]<<", "<<q_init[1]<< ", loc "<<temp[0]<<", "<<temp[1]<<"\n";
        

        //check to see if q_init in collision?


        amp::Path2D path = myWaveFront::planInCSpace(q_init_wrap, q_goal_wrap, grid_cspace, true);
        amp::unwrapPath(path,Eigen::Vector2d(0.0,0.0), Eigen::Vector2d(2.0*3.1415,2.0*3.1415));
        return path;
    }

    // virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override{
    //     return myWaveFront::constructDiscretizedWorkspace(environment,_cell_width);
    // }

    private:

    double _cell_width;

};