
#include "AMPCore.h"
namespace Rotate{

/// @brief the angle of the line (x axis=0, positive in ccw dir) defined by two points 
inline double ang02pi(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
    double ang = atan2(p2[1]-p1[1],p2[0]-p1[0]);
    if (ang<0){return ang+(2*3.1415);}
    else return ang;
}

/// @brief the angle of the line (x axis=0, positive in ccw dir) defined by two points 
inline double ang(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
    return atan2(p2[1]-p1[1],p2[0]-p1[0]);
}

}
