#include "Arm2L.h"

Eigen::Vector2d Arm2L::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const{
    return Eigen::Vector2d();
}


amp::ManipulatorState Arm2L::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
    return amp::ManipulatorState ();
}
