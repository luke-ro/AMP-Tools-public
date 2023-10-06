#include "Arm2L.h"

Eigen::Vector2d Arm2L::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const{
    Eigen::Vector2d loc = m_base_location;

    // Go through links to get nth joint location
    for(int i =0; i<joint_index; i++){
        loc[0] += m_link_lengths[i]*cos(state[i]);
        loc[1] += m_link_lengths[i]*sin(state[i]);
    }

    return loc;
}


amp::ManipulatorState Arm2L::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
    return amp::ManipulatorState ();
}
