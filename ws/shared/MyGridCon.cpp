#include "MyGridCon.h"

std::unique_ptr<amp::GridCSpace2D> MyGridCon::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
            // makes a CSpace2D object and initializes it with bounds from env
            // std::unique_ptr<amp::GridCSpace2D> cspace(new CSpace2D(env.x_min,env.x_max,env.y_min,env.y_max));
            
            auto ptr = std::make_unique<CSpace2D>(env.x_min,env.x_max,env.y_min,env.y_max);
            // std::unique_ptr<amp::ConfigurationSpace2D> ptr = &cspace;
            // cspace->in
            CSpace2D temp(env.x_min,env.x_max,env.y_min,env.y_max);
            ptr = temp.genCSpace(manipulator,env); //genCSpace(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);
            // std::unique_ptr<amp::ConfigurationSpace2D> n_ptr(ptr);
            return ptr;
        }