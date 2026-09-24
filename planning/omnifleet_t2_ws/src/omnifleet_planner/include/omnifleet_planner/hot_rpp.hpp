#pragma once
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
namespace omnifleet_planner {
class HotRegulatedPurePursuitController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override {
    syncAdditionalParameters();
    return RegulatedPurePursuitController::computeVelocityCommands(pose, speed, checker);
  }
protected:
  void syncAdditionalParameters() {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("RPP parameter node expired");}
    std::lock_guard<std::mutex> lock(mutex_);
    node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
    node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  }
};
}
