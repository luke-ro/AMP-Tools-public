#include "QuadAgentProperties.h"

// enum States {STATEu,STATE}



Eigen::Matrix<double,6,1> QuadAgentProperties::dynamics(Eigen::Matrix<double,6,1> x, Eigen::Vector2d control) const{
    // # x = x[0] # horizontal position inertial (not used)
    // # z = x[1] # vertical position inertial (not used)
    double th = x[2]; // pitch 
    double u = x[3]; // horz velocity body
    double w = x[4]; //# vert velocity body 
    double q = x[5]; //# pitch rate

    //controls
    double Fz = control[0]; // control force in z body
    double Fx = 0.0;
    double pitch_moment = control[1]; //# control moment about y 

    Eigen::Matrix<double,6,1> dx;
    dx[0] = (cos(th)*u) + (sin(th)*w);
    dx[1] = -sin(th)*u + cos(th)*w;
    dx[2] = q;
    dx[3] = -q*w + Fx/m_tot  - g*sin(th);
    dx[4] = q*u + Fz/m_tot + g*cos(th);
    dx[5] = pitch_moment/Iy;

    return dx;
}