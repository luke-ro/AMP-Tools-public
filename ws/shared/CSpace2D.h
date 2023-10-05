#include "AMPCore.h"
#include <math.h>

class CSpace2D : public amp::ConfigurationSpace2D {
    public:
        CSpace2D(double x0_min, double x0_max, double x1_min, double x1_max);
        
        amp::Polygon minkDiff(const amp::Polygon& obs,  const amp::Polygon& rob_pos);
        amp::Polygon pgDifference(const amp::Polygon& pg);
        amp::Polygon reorderPGCCW(const amp::Polygon& pg, bool zero=false);
        int botLeftVerCCW(const amp::Polygon& pg);

        bool inCollision(double x0, double x1) const override{
            return true;
        }

        /// @brief the angle of the line (x axis=0, positive in ccw dir) defined by two points 
        inline double ang02pi(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2){
            double ang = atan2(p2[1]-p1[1],p2[0]-p1[0]);
            if (ang<0){return ang+(2*3.1415);}
            else return ang;
        }


};