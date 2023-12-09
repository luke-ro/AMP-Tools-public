#pragma once

#include "AMPCore.h"
#include <vector>
#include <math.h>

typedef Eigen::Matrix<double,6,1> QuadState;
typedef Eigen::Matrix<double,6,1> QuadDerivativeState;
typedef std::vector<QuadState> QuadAgentTrajectory;
typedef std::vector<QuadAgentTrajectory> QuadAgentsTrajectories; 

class QuadAgentProperties{
    public:
    QuadAgentProperties(QuadState q_init_=QuadState(), QuadState q_goal_=QuadState(), double l_arm_=.15, double m_arm_=.05, double m_motor_=05):
    q_init (q_init_),
    q_goal (q_goal_),
    l_arm(l_arm_),
    m_arm(m_arm_),
    m_motor(m_motor_),
    m_tot(m_arm_+m_motor_),
    Iy(2.0*m_arm_*pow(l_arm_,2) + (m_arm_*pow(2*l_arm_,2))/12),
    radius(2*l_arm_),
    max_motor_thrust(3)
    {}

    Eigen::Matrix<double,6,1> dynamics(Eigen::Matrix<double,6,1> x, Eigen::Vector2d control) const;
    // Eigen::Vector2d randomControl();
    // Eigen::Vector2d motorCommandsToControl(Eigen::Vector2d motor_commands);
    // Eigen::Matrix<double,6,1> getState(){return _x;}

    const QuadState q_init;
    const QuadState q_goal;
    const double l_arm; // [m] length of quad arm
    const double m_arm; // [kg] mass of arm
    const double m_motor; // [kg] mass of one motor
    const double m_tot; // [kg]
    const double Iy; // [kg/m^2 ?]
    const double radius; // [m]
    const double max_motor_thrust; //[N]
    const double max_vel = 20; // [m/s]
    const double max_pitch_rate = 2.0*(2.0*3.1415); // [1/s]
    const double g = 9.81; // [m/s]
};

struct QuadAgentProblem{
    amp::Environment2D env;

    int num_agents = 0;
    std::vector<QuadAgentProperties> agents;
};