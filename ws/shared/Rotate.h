#pragma once

#include "AMPCore.h"

#define PI 3.1415

namespace Rotate{


/// @brief the angle of the line (x axis=0, positive in ccw dir) defined by two points 
static double ang02pi(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
    double ang = atan2(p2[1]-p1[1],p2[0]-p1[0]);
    if (ang<0){return ang+(2*3.1415);}
    else return ang;
}

/// @brief the angle of the line (x axis=0, positive in ccw dir) defined by two points 
inline double ang(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
    return atan2(p2[1]-p1[1],p2[0]-p1[0]);
}

/// @brief returns the difference from [-pi,pi) between two angles 
inline double angleDifference(double a1, double a2){      
    double a = a1 - a2;
    if (a>=PI){
        return a-(2*PI);
    }else if(a<-PI){
        return a+(2*PI);
    } 
    return a;
}

inline void rotateLine2DInPlace(Eigen::Vector2d& p1, Eigen::Vector2d& p2, double angle, const Eigen::Vector2d pivot){
    Eigen::Matrix3d R;
    Eigen::Matrix3d T1;
    Eigen::Matrix3d T2;

    T1 << 1,  0,  -pivot[0],
          0,  1,  -pivot[1],
          0,  0,  1;

    R <<  cos(angle),  -sin(angle),  0,
         sin(angle),  cos(angle),  0,
          0,           0,           1;


    T2 << 1,  0,  pivot[0],
          0,  1,  pivot[1],
          0,  0,  1;
    
    Eigen::Matrix3d T = T2*R*T1;

    Eigen::Vector3d u = {p1[0], p1[1], 1};
    Eigen::Vector3d v = {p2[0], p2[1], 1};

    u = T*u;
    v = T*v;

    p1[0] = u[0];
    p1[1] = u[1];

    p2[0] = v[0];
    p2[1] = v[1];
}

inline Eigen::Vector2d rotatePoint(const Eigen::Vector2d& p1, double angle, const Eigen::Vector2d& pivot){
    Eigen::Matrix3d R;
    Eigen::Matrix3d T1;
    Eigen::Matrix3d T2;

    T1 << 1,  0,  -pivot[0],
          0,  1,  -pivot[1],
          0,  0,  1;

    R <<  cos(angle), -sin(angle),  0,
          sin(angle),  cos(angle),  0,
          0,           0,           1;


    T2 << 1,  0,  pivot[0],
          0,  1,  pivot[1],
          0,  0,  1;
    
    Eigen::Matrix3d T = T2*R*T1;

    Eigen::Vector3d u = {p1[0], p1[1], 1};
    u = T*u;

    Eigen::Vector2d temp {u[0], u[1]};
    return temp;
}

inline amp::Polygon rotatePG(amp::Polygon pg, double angle, Eigen::Vector2d pivot){
    Eigen::Matrix3d R;
    Eigen::Matrix3d T1;
    Eigen::Matrix3d T2;

    T1 << 1,  0,  -pivot[0],
          0,  1,  -pivot[1],
          0,  0,  1;

    R <<  cos(angle), -sin(angle),  0,
          sin(angle),  cos(angle),  0,
          0,           0,           1;


    T2 << 1,  0,  pivot[0],
          0,  1,  pivot[1],
          0,  0,  1;
    
    Eigen::Matrix3d T = T2*R*T1;
    
    std::vector<Eigen::Vector2d> verts;
    for(auto vert: pg.verticesCCW()){
        Eigen::Vector3d u;
        u << vert[0], vert[1], 1;
        Eigen::Vector3d v = T*u;
        Eigen::Vector2d w;
        w << v[0], v[1];
        verts.push_back(w);
    }

    return amp::Polygon(verts);
}

}
