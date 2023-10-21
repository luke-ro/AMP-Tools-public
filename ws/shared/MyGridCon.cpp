#include "MyGridCon.h"

std::unique_ptr<amp::GridCSpace2D> MyGridCon::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){

            // makes a CSpace2D object and initializes it with bounds from env
            // std::unique_ptr<CSpace2D> ptr(new CSpace2D(env.x_min,env.x_max,env.y_min,env.y_max, _n1, _n2));
            std::unique_ptr<CSpace2D> ptr(new CSpace2D(0.0,2*3.1415,0.0,2*3.1415, _n1, _n2));
            // CSpace2D cspace(env.x_min,env.x_max,env.y_min,env.y_max);
            CSpace2D temp = ptr->genCSpace(manipulator,env);
            
            for(int i=0;i<_n1;i++){
                for(int j=0;j<_n2;j++){
                    (*ptr)(i,j) = temp(i,j);
                }
            }
            // return ptr;
            return ptr;
        }