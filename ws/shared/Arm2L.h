#include "AMPCore.h"

class Arm2L : public amp::LinkManipulator2D {
    public: 
        Arm2L();

        Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;
        amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
};