#pragma once 

#include <iostream>

#include "AMPCore.h"

#include "Helpers.h"
#include "Rotate.h"
#include "QuadAgentProperties.h"


namespace QuadAgentTools{
    static Eigen::Vector2d motorCommandsToControl(const QuadAgentProperties& agent, Eigen::Vector2d motor_commands);

    /**
     * @brief returns a random control input, Z (vert force), M (pitching moment)
    */
    static Eigen::Vector2d randomControl(const QuadAgentProperties& agent, const QuadState& x0, bool limit=0);

    static uint32_t getNearestNeighbor(std::vector<QuadState> node_vec, const QuadState& state);

    /**
     * @brief samples the space given 
    */
    static QuadState sampleSpace(amp::Environment2D env, QuadAgentProperties agent);
    
    static QuadState steer(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& q0, const QuadState& q_steer, double Dt, int n);
    static QuadState steer(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& q0, const QuadState& q_steer, double Dt, int n, Eigen::Vector2d& motor_control);

    static Eigen::Matrix<double,6,1> rk4(const QuadAgentProperties& agent, const QuadState& y0, const Eigen::Vector2d& u, double dt);

    static double distFunc(QuadState q0, QuadState q1);

    static Eigen::Vector2d getPos(const QuadState& state);

    
    static bool withinBounds(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& x);

    static void printState(QuadState x){
        std::cout<<x[0]<<", "<<x[1]<<", "<<x[2]<<", "<<x[3]<<", "<<x[4]<<", "<<x[5]<<"\n";
    }

    static double clip(double q, double low, double high);    
}

static double QuadAgentTools::clip(double q, double low, double high){
    double res;
    if (q>high)
        return high;

    if(q<low)
        return low;

    return q;
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
static Eigen::Vector2d QuadAgentTools::randomControl(const QuadAgentProperties& agent, const QuadState& x0, bool limit){
    double thrust = amp::RNG::randd(0,2.0*agent.max_motor_thrust);
    // double thrust = amp::RNG::nrand()*2.0*agent.max_motor_thrust;
    double diff = amp::RNG::nrand()*0.3;
    Eigen::Vector2d motor_commands(thrust*0.5-diff, thrust*0.5+diff);

    // use derivitive controller to dampen the rotational velocity
    if(limit && abs(x0[5])>2.0){
        double moment_des = -agent.kd*x0[5];

        Eigen::Matrix2d A;
        A<<-1.0,-1.0, -agent.l_arm, agent.l_arm;
        Eigen::Vector2d control_des(thrust,moment_des);
        Eigen::Matrix2d Ainv = A.inverse();
        // Eigen::Matrix2d Ainv;
        
        Eigen::Vector2d forces_des = Ainv*control_des;
        motor_commands[0] = forces_des[0];
        motor_commands[1] = forces_des[1];
    }

    motor_commands[0] = clip(motor_commands[0], 0, agent.max_motor_thrust);
    motor_commands[1] = clip(motor_commands[1], 0, agent.max_motor_thrust);
    // Eigen::Vector2d motor_commands(amp::RNG::randd(0, agent.max_motor_thrust),amp::RNG::randd(0, agent.max_motor_thrust));
    Eigen::Vector2d control = motorCommandsToControl(agent, motor_commands);
    // std::cout << control[0] << ", " << control[1] <<", "<< motor_commands(0) << ", " << motor_commands(1) << "\n";

    return control;
}

/**
 * @brief returns a randomly sampled state
*/
static QuadState QuadAgentTools::sampleSpace(amp::Environment2D env, QuadAgentProperties agent){
    // std::uniform_real_distribution<double> x_pos(env.x_min, env.x_max);
    // std::uniform_real_distribution<double> y_pos(env.y_min, env.y_max);
    // std::uniform_real_distribution<double> theta(0.0, 2.0*3.1415);
    // std::uniform_real_distribution<double> vel_mag(-agent.max_vel, agent.max_vel);
    // std::uniform_real_distribution<double> pitch_rate(-agent.max_pitch_rate, agent.max_pitch_rate);

    // std::default_random_engine re;

    Eigen::Vector2d vel(amp::RNG::randf(-agent.max_vel,agent.max_vel), 0.0); // generate random vel magnitude
    vel = Rotate::rotatePoint(vel,amp::RNG::randf(0.0,3.1415),Eigen::Vector2d()); // split that vel into random direction

    QuadState rand_state;
    rand_state(0) = amp::RNG::randf(env.x_min, env.x_max);
    rand_state(1) = amp::RNG::randf(env.y_min, env.y_max);
    rand_state(2) = amp::RNG::randf(0.0, 2*3.1415);
    rand_state(3) = vel(0);
    rand_state(4) = vel(1);
    rand_state(5) = amp::RNG::randf(-agent.max_pitch_rate, agent.max_pitch_rate);
    
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


static QuadState QuadAgentTools::steer(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& q0, const QuadState& q_steer, double Dt, int n){
    Eigen::Vector2d trash_val;
    return steer(env, agent, q0, q_steer, Dt, n, trash_val);
}

/**
 * @brief Samples random control inputs to get trajectory
*/
static QuadState QuadAgentTools::steer(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& q0, const QuadState& q_steer, double Dt, int n, Eigen::Vector2d& control_used){
    int n_max=n;

    // initialize and get min states
    Eigen::Vector2d control_rand = randomControl(agent, q0);
    Eigen::Vector2d control_min = control_rand;
    QuadState q_min = rk4(agent, q0, control_rand, Dt);
    double min_dist = distFunc(q_min,q_steer);
    
    // std::cout<<"---------------------------\n";
    // std::cout<<"q0: ";
    // printState(q0);
    // std::cout<<"q_steer: ";
    // printState(q_steer);
    // std::cout<<"samples:\n ";
    // printState(q_min);

    QuadState q_sample;
    for(int i=0; i<n_max; i++){

        // random sample 
        control_rand = randomControl(agent, q0);
        q_sample = rk4(agent,q0,control_rand,Dt);
        double cur_dist = distFunc(q_sample,q_steer);

        // printState(q_sample);

        // check for new min
        if(cur_dist<min_dist && withinBounds(env,agent,q_sample)){
            min_dist = cur_dist;
            q_min = q_sample;
            control_min = control_rand;
        }

    }

    // std::cout<<"returned: ";
    // printState(q_min);

    control_used = control_min;
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

/**
 * @brief checks if a circular agent is within bounds
*/
static bool QuadAgentTools::withinBounds(const amp::Environment2D& env, const QuadAgentProperties& agent, const QuadState& x){
    if(x(0) - agent.radius < env.x_min)
        return false;
    if(x(0) + agent.radius > env.x_max)
        return false;
    if(x(1) - agent.radius < env.y_min)
        return false;
    if(x(1) + agent.radius > env.y_max)
        return false;

    return true;
}