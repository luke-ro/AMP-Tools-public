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
}