#include "AMPCore.h"
#include <vector>
#include <math.h>

typedef Eigen::Matrix<double,6,1> QuadState;
typedef Eigen::Matrix<double,6,1> QuadDerivativeState;
typedef std::vector<QuadState> QuadAgentTrajectory;
typedef std::vector<QuadAgentTrajectory> QuadAgentsTrajectories; 

class QuadAgentProperties{
    public:
    QuadAgentProperties(double l_arm_=.15, double m_arm_=.05, double m_motor_=05):
    l_arm(l_arm_),
    m_arm(m_arm_),
    m_motor(m_motor_),
    m_tot(m_arm_+m_motor_),
    Iy(2.0*m_arm_*pow(l_arm_,2) + (m_arm_*pow(2*l_arm_,2))/12),
    radius(2*l_arm_),
    max_motor_thrust(2.0)
    {}

    Eigen::Matrix<double,6,1> dynamics(Eigen::Matrix<double,6,1> x, Eigen::Vector2d control);
    Eigen::Vector2d randomControl();
    Eigen::Vector2d motorCommandsToControl(Eigen::Vector2d motor_commands);
    // Eigen::Matrix<double,6,1> getState(){return _x;}

    const double l_arm; // [m] length of quad arm
    const double m_arm; // [kg] mass of arm
    const double m_motor; // [kg] mass of one motor
    const double m_tot;
    const double Iy;
    const double radius;
    const double max_motor_thrust;
    const double g = 9.81;
};

struct QuadAgentProblem{
    amp::Environment2D env;

    int num_agents = 0;
    std::vector<QuadAgentProperties> agents;
    std::vector<Eigen::Matrix<double,6,1>> start_states;
};