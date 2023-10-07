#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include <math.h>
#include "Helpers.h"



class CSpace2D : public amp::GridCSpace2D {
    public:
        CSpace2D(double x0_min, double x0_max, double x1_min, double x1_max, int s0=100, int s1=100);

        // returns true if val is between the values specified by x1 and x2 exlusive
        bool isBetwOpen(double val, double x1, double x2){
            return (val>std::min(x1,x2)) && (val<std::max(x1,x2));
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

        inline double idxToNumx0(int idx){ return double(idx)/x0_len*(m_x0_bounds.second-m_x0_bounds.first);}
        inline double idxToNumx1(int idx){ return double(idx)/x1_len*(m_x1_bounds.second-m_x1_bounds.first);}


        amp::DenseArray2D<bool> c_arr;
        const int x0_len;
        const int x1_len;

        ~CSpace2D(){}

};
