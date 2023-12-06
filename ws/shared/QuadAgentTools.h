#pragma once 
#include "AMPCore.h"

#include "Helpers.h"
#include "Rotate.h"
#include "QuadAgentProperties.h"

namespace QuadAgentTools{
    inline Eigen::Vector2d motorCommandsToControl(const QuadAgentProperties& agent, Eigen::Vector2d motor_commands);

    /**
     * @brief returns a random control input, Z (vert force), M (pitching moment)
    */
    inline Eigen::Vector2d randomControl(const QuadAgentProperties& agent);

    /**
     * @brief samples the space given 
    */
    inline QuadState sampleSpace(amp::Environment2D env, QuadAgentProperties agent);

    inline QuadState steer(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& q0, const QuadState& q_steer, double Dt);

    Eigen::Matrix<double,6,1> rk4(const QuadAgentProperties& agent, const QuadState& y0, const Eigen::Vector2d& u, double dt);

    double distFunc(QuadState q0, QuadState q1);


}