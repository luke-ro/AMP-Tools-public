
#include "QuadAgentTools.h"

inline Eigen::Vector2d QuadAgentTools::motorCommandsToControl(const QuadAgentProperties& agent, Eigen::Vector2d motor_commands){
    Eigen::Vector2d control;
    control[0] = -(motor_commands[0] + motor_commands[1]);
    control[1] = agent.l_arm*(motor_commands[1]-motor_commands[0]);
    return control;
}

/**
 * @brief returns a random control input, Z (vert force), M (pitching moment)
*/
inline Eigen::Vector2d QuadAgentTools::randomControl(const QuadAgentProperties& agent){
    std::uniform_real_distribution<double> dist(0,agent.max_motor_thrust);
    std::default_random_engine re;
    
    Eigen::Vector2d motor_commands(dist(re),dist(re));
    Eigen::Vector2d control = motorCommandsToControl(agent, motor_commands);
    return control;
}

/**
 * @brief returns a randomly sampled state
*/
inline QuadState QuadAgentTools::sampleSpace(amp::Environment2D env, QuadAgentProperties agent){
    std::uniform_real_distribution<double> x_pos(env.x_min, env.x_max);
    std::uniform_real_distribution<double> y_pos(env.y_min, env.y_max);
    std::uniform_real_distribution<double> theta(0.0, 2.0*3.1415);
    std::uniform_real_distribution<double> vel_mag(-agent.max_vel, agent.max_vel);
    std::uniform_real_distribution<double> pitch_rate(-agent.max_pitch_rate, agent.max_pitch_rate);

    std::default_random_engine re;

    Eigen::Vector2d vel(vel_mag(re),0); // generate random vel magnitude
    vel = Rotate::rotatePoint(vel,theta(re),Eigen::Vector2d()); // split that vel into random direction

    QuadState rand_state;
    rand_state(0) = x_pos(re);
    rand_state(1) = y_pos(re);
    rand_state(2) = theta(re);
    rand_state(3) = vel(0);
    rand_state(4) = vel(1);
    rand_state(5) = pitch_rate(re);
    
    return rand_state;
}