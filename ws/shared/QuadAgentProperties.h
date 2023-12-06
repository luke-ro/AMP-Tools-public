#include "AMPCore.h"
#include <vector>
#include <math.h>

typedef std::vector<Eigen::Matrix<double,6,1>> QuadAgentTraj;
typedef std::vector<QuadAgentTraj> QuadAgentsTrajectories; 

class QuadAgentProperties{
    public:
    QuadAgentProperties(double l_arm=.15, double m_arm=.05, double m_motor=05):
    _l_arm(l_arm),
    _m_arm(m_arm),
    _m_motor(_m_motor),
    _m_tot(m_arm+m_motor),
    _Iy(2.0*m_arm*pow(l_arm,2) + (m_arm*pow(2*l_arm,2))/12),
    _radius(2*l_arm),
    _max_motor_thrust(2.0)
    {}

    Eigen::Matrix<double,6,1> dynamics(Eigen::Matrix<double,6,1> x, Eigen::Vector2d control);
    Eigen::Vector2d motorCommmandsToControl(Eigen::Vector2d motor_commands);
    // Eigen::Matrix<double,6,1> getState(){return _x;}

    const double _l_arm; // [m] length of quad arm
    const double _m_arm; // [kg] mass of arm
    const double _m_motor; // [kg] mass of one motor
    const double _m_tot;
    const double _Iy;
    const double _radius;
    const double _max_motor_thrust;
    const double _g = 9.81;
};

struct QuadAgentProblem{
    amp::Environment2D env;

    int num_agents = 0;
    std::vector<QuadAgentProperties> agents;
    std::vector<Eigen::Matrix<double,6,1>> start_states;
};