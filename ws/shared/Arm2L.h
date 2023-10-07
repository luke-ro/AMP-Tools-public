#include "AMPCore.h"
#include "Rotate.h"

class Arm2L : public amp::LinkManipulator2D {
    public: 
        Arm2L() : LinkManipulator2D() {}

        Arm2L(const std::vector<double>& link_lengths);

        Arm2L(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths);

        Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;
        amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
};