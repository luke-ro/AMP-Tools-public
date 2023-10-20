#include "MyGridCon.h"

std::unique_ptr<amp::GridCSpace2D> MyGridCon::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
            // makes a CSpace2D object and initializes it with bounds from env
            std::unique_ptr<CSpace2D> ptr(new CSpace2D(env.x_min,env.x_max,env.y_min,env.y_max, 100, 100));
            // CSpace2D cspace(env.x_min,env.x_max,env.y_min,env.y_max);
            ptr->genCSpace(manipulator,env);
            // return ptr;
            return ptr;
        }