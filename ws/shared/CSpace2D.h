#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <math.h>
// #include "Helpers.h"



class CSpace2D : public amp::GridCSpace2D {
    public:
        CSpace2D(double x0_min, double x0_max, double x1_min, double x1_max, int s0=100, int s1=100);

        // returns true if val is between the values specified by x1 and x2 exlusive
        bool isBetwOpen(double val, double x1, double x2){
            return (val>std::min(x1,x2)) && (val<std::max(x1,x2));
        }

        // returns true if val is between the values specified by x1 and x2 inclusive
        bool isBetwClosed(double val, double x1, double x2){
            return (val>=std::min(x1,x2)) && (val<=std::max(x1,x2));
        }

        // returns true if val is between the values specified by x1(inclusive) and x2 (exclusive)
        bool isBetwLeftClosed(double val, double x1, double x2){
            if (x1<x2){
                return (val>=x1) && (val<x2);
            }else if(x1>x2){
                return (val>x2) && (val<=x1);
            }else{
                return val==x1;
            }
        }

        bool insidePolygon(const amp::Polygon& pg, const Eigen::Vector2d& q){
                //reset number of intersections for each obstacle
            int num_intersections = 0;

            int num_verts = pg.verticesCCW().size();
            
            Eigen::Vector2d p1,p2;
            // printf("`insidePolygon` AT %2.2f, %2.2f", q[0],q[1]);
            //iterate through edges
            for(int i=0;i<num_verts;i++){
                // get the current edge's vertices 
                p1 = pg.verticesCCW()[i];
                p2 = pg.verticesCCW()[(i+1)%num_verts]; //gets the next vertex (and wraps to the begining)

                // calculate the line for the edge
                double m = (p2[1]-p1[1])/(p2[0]-p1[0]);
                double x_inter = (q[1]-p1[1])/m + p1[0];
                auto f_of_x = [m,p1](double x){return (m*(x-p1[0]))+p1[1];}; 

                // only checking for intersections on the right of the point
                if (m!=0 && x_inter<q[0]){
                    // intersection occurs to the left of q[0]. 
                    // Pass.

                //check to see if point lies on the edge. Return true if so.
                }else if(q[1]==f_of_x(q[0]) && isBetwClosed(q[0],p1[0],p2[0])){
                    return true;
                
                // slope of zero (horiz) edge condition
                // }else if(m==0){
                //     // horizontal lines dont count as intersection. 
                //     // Pass.

                // inf slope (vert) edge condition
                }else if(std::isinf(m) || std::isinf(-m)){
                    if(q[0] <= p1[0] && isBetwLeftClosed(q[1],p1[1],p2[1])){
                        // point is to the left inclusive and is vertically within bounds
                        num_intersections++;
                    }

                // check the first vertex. If same y (and to the left exclusive), count as intersection
                }else if(q[1]==p1[1] && q[0]>p1[1]){
                    //might be hnadled by the last if statement?
                    num_intersections++;        
                

                // check if the x intersept is between the two x coords of points
                }else if(isBetwLeftClosed(x_inter,p1[0],p2[0])){
                    num_intersections++;
                }
            }

            // odd number of intersections means inside obstacle
            if(num_intersections%2==1)
                return true;
            return false;
        }

        /**
         * Checks entire workspace for collision
         * 
         * @param env the dataframe that contains the workspace ans obstacles
         * @param q the 2D point to check
        */
        bool checkCollsionEnv(const amp::Environment2D& env, const Eigen::Vector2d& q){
            // iterate through obstacles
            for(auto obs : env.obstacles){
                if (insidePolygon(obs,q)){
                    //collision was found
                    // LOG("COLLISION: "<< std::format("{}",x[0]) << ", " << std::format("{}",x[1]));
                    // printf("COLLISION AT %2.2f, %2.2f", x[0],x[1]);
                    return true;
                }
            }

            // if at this point, a collision was not found
            return false;
        }

        /*
        *  MATLAB or Numpy linspace like function. 
        *  taken from https://stackoverflow.com/questions/27028226/python-linspace-in-c
        */
        template<typename T>
        std::vector<double> linspace(T start_in, T end_in, int num_in)
        {

        std::vector<double> linspaced;

        double start = static_cast<double>(start_in);
        double end = static_cast<double>(end_in);
        double num = static_cast<double>(num_in);

        if (num == 0) { return linspaced; }
        if (num == 1) 
            {
            linspaced.push_back(start);
            return linspaced;
            }

        double delta = (end - start) / (num - 1);

        for(int i=0; i < num-1; ++i)
            {
            linspaced.push_back(start + delta * i);
            }
        linspaced.push_back(end); // I want to ensure that start and end
                                    // are exactly the same as the input
        return linspaced;
        }

        std::vector<Eigen::Vector2d> linspace2D(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, int n){
            std::vector<double> x1 = linspace(p1[0],p2[0],n);
            std::vector<double> x2 = linspace(p1[1],p2[1],n);

            std::vector<Eigen::Vector2d> line;
            Eigen::Vector2d temp;
            
            for(int i=0; i<n; i++){
                temp[0] = x1[i];
                temp[1] = x2[i];
                line.push_back(temp);
            }
            return line;
        }

        
        amp::Polygon minkDiff(const amp::Polygon& obs,  const amp::Polygon& rob_pos);
        amp::Polygon pgDifference(const amp::Polygon& pg);
        amp::Polygon reorderPGCCW(const amp::Polygon& pg, bool zero=false);
        int botLeftVerCCW(const amp::Polygon& pg);
        CSpace2D genCSpace(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);
        // amp::DenseArray2D getCSpace(){return c_arr;};

        bool inCollision(double x0, double x1) const override;
        bool checkCollision(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env, double theta1, double theta2);
        bool checkColConfig(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env, double theta1, double theta2);
        

        /// @brief the angle of the line (x axis=0, positive in ccw dir) defined by two points 
        inline double ang02pi(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
            double ang = atan2(p2[1]-p1[1],p2[0]-p1[0]);
            if (ang<0){return ang+(2*3.1415);}
            else return ang;
        }

        inline int numToIdx0(double num){ return num/(m_x0_bounds.second-m_x0_bounds.first)*x0_len;}
        inline int numToIdx1(double num){ return num/(m_x1_bounds.second-m_x1_bounds.first)*x1_len;}

        inline double idxToNumx0(int idx){ return idx/x0_len*(m_x0_bounds.second-m_x0_bounds.first);}
        inline double idxToNumx1(int idx){ return idx/x1_len*(m_x1_bounds.second-m_x1_bounds.first);}


        amp::DenseArray2D<bool> c_arr;
        const int x0_len;
        const int x1_len;

        ~CSpace2D(){}

};

class MyGridCon : public amp::GridCSpace2DConstructor {
    public:
    
    virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override{
        // makes a CSpace2D object and initializes it with bounds from env
            // std::unique_ptr<amp::GridCSpace2D> cspace(new CSpace2D(env.x_min,env.x_max,env.y_min,env.y_max));
            
            auto ptr = std::make_unique<CSpace2D>(env.x_min,env.x_max,env.y_min,env.y_max);
            // std::unique_ptr<amp::ConfigurationSpace2D> ptr = &cspace;
            // cspace->in
            ptr->genCSpace(manipulator,env); //genCSpace(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);
            // std::unique_ptr<amp::ConfigurationSpace2D> n_ptr(ptr);
            return ptr;
        
    }
};


// std::unique_ptr<amp::GridCSpace2D> MyGridCon::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override{
//             // makes a CSpace2D object and initializes it with bounds from env
//             // std::unique_ptr<amp::GridCSpace2D> cspace(new CSpace2D(env.x_min,env.x_max,env.y_min,env.y_max));
            
//             auto ptr = std::make_unique<CSpace2D>(env.x_min,env.x_max,env.y_min,env.y_max);
//             // std::unique_ptr<amp::ConfigurationSpace2D> ptr = &cspace;
//             // cspace->in
//             ptr->genCSpace(manipulator,env); //genCSpace(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);
//             // std::unique_ptr<amp::ConfigurationSpace2D> n_ptr(ptr);
//             return ptr;
//         }