#include "QuadAgent.h"

// enum States {STATEu,STATE}



Eigen::Matrix<double,6,1> QuadAgent::dynamics(Eigen::Matrix<double,6,1> x, Eigen::Vector3d control){
    // # x = x[0] # horizontal position inertial (not used)
    // # z = x[1] # vertical position inertial (not used)
    double th = x[2]; // pitch 
    double u = x[3]; // horz velocity body
    double w = x[4]; //# vert velocity body 
    double q = x[5]; //# pitch rate

    //controls
    double Fx = control[0]; //# force in x body
    double Fz = control[1]; // force in z body
    double pitch_moment = control[2]; //# moment about y 

    Eigen::Matrix<double,6,1> dx;
    dx[0] = (cos(th)*u) + (sin(th)*w);
    dx[1] = -sin(th)*u + cos(th)*w;
    dx[2] = q;
    dx[3] = -q*w + Fx/_m_tot  - _g*sin(th);
    dx[4] = q*u + Fz/_m_tot + _g*cos(th);
    dx[5] = pitch_moment/_Iy;

    return dx;
}