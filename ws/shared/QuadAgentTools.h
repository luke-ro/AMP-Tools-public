#pragma once 
#include "AMPCore.h"

#include "Helpers.h"
#include "Rotate.h"
#include "QuadAgentProperties.h"

namespace QuadAgentTools{
    static Eigen::Vector2d motorCommandsToControl(const QuadAgentProperties& agent, Eigen::Vector2d motor_commands);

    /**
     * @brief returns a random control input, Z (vert force), M (pitching moment)
    */
    static Eigen::Vector2d randomControl(const QuadAgentProperties& agent);

    static uint32_t getNearestNeighbor(std::vector<QuadState> node_vec, const QuadState& state);

    /**
     * @brief samples the space given 
    */
    static QuadState sampleSpace(amp::Environment2D env, QuadAgentProperties agent);

    static QuadState steer(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& q0, const QuadState& q_steer, double Dt);

    static Eigen::Matrix<double,6,1> rk4(const QuadAgentProperties& agent, const QuadState& y0, const Eigen::Vector2d& u, double dt);

    static double distFunc(QuadState q0, QuadState q1);

    static Eigen::Vector2d getPos(const QuadState& state);

}


static Eigen::Vector2d QuadAgentTools::getPos(const QuadState& state){
    return Eigen::Vector2d(state(0),state(1));
}


static Eigen::Vector2d QuadAgentTools::motorCommandsToControl(const QuadAgentProperties& agent, Eigen::Vector2d motor_commands){
    Eigen::Vector2d control;
    control[0] = -(motor_commands[0] + motor_commands[1]);
    control[1] = agent.l_arm*(motor_commands[1]-motor_commands[0]);
    return control;
}

/**
 * @brief returns a random control input, Z (vert force), M (pitching moment)
*/
static Eigen::Vector2d QuadAgentTools::randomControl(const QuadAgentProperties& agent){
    std::uniform_real_distribution<double> dist(0,agent.max_motor_thrust);
    std::default_random_engine re;
    
    Eigen::Vector2d motor_commands(dist(re),dist(re));
    Eigen::Vector2d control = motorCommandsToControl(agent, motor_commands);
    return control;
}

/**
 * @brief returns a randomly sampled state
*/
static QuadState QuadAgentTools::sampleSpace(amp::Environment2D env, QuadAgentProperties agent){
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

/**
 * @brief gets nearest nerighbor according to L2 norm of position
*/
static uint32_t QuadAgentTools::getNearestNeighbor(std::vector<QuadState> node_vec, const QuadState& state){
    uint32_t nearest = 0;
    double min_dist = std::numeric_limits<double>::max();
    for(int i=0; i<node_vec.size(); i++){
        // double dist = (p2-p_query).norm();
        double dist = distFunc(state,node_vec[i]);
        if(dist<min_dist && dist>0){ //check for >0 to not return same point
            nearest = i;
            min_dist = dist;
        }
    }
    return nearest;
}

/**
 * @brief Samples random control inputs to get trajectory
*/
static QuadState QuadAgentTools::steer(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& q0, const QuadState& q_steer, double Dt){
    int n_max=10;
    Eigen::Vector2d control_rand = randomControl(agent);
    QuadState q_min =  rk4(agent,q0,control_rand,Dt);
    double min_dist = distFunc(q_min,q_steer);

    QuadState q_sample;
    for(int i=0; i<n_max; i++){
        control_rand = randomControl(agent);
        q_sample = rk4(agent,q0,control_rand,Dt);
        double cur_dist = distFunc(q_sample,q_steer);
        if(cur_dist<min_dist){
            min_dist = cur_dist;
            q_min = q_sample;
        }

    }
    return q_min;
}

static Eigen::Matrix<double,6,1> QuadAgentTools::rk4(const QuadAgentProperties& agent, const QuadState& y0, const Eigen::Vector2d& u, double dt){
    Eigen::Matrix<double,6,1> k1 = agent.dynamics(y0, u);
    Eigen::Matrix<double,6,1> k2 = agent.dynamics(y0+dt*k1/2, u);
    Eigen::Matrix<double,6,1> k3 = agent.dynamics(y0+dt*k2/2, u);
    Eigen::Matrix<double,6,1> k4 = agent.dynamics(y0+dt*k3, u);

    Eigen::Matrix<double,6,1> y1 = y0 + (dt/6) * (k1 + (2*k2) + (2*k3) + k4);
    return y1;
}


static double QuadAgentTools::distFunc(QuadState q0, QuadState q1){
    Eigen::Vector2d p0,p1;
    p0<<q0(0),q0(1);
    p1<<q1(0),q1(1);
    return (p1-p0).norm();
}