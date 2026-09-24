#include <gtest/gtest.h>
#include "omnifleet_planner/hot_rpp.hpp"
class RppProbe : public omnifleet_planner::HotRegulatedPurePursuitController {
public:
  void setup(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) {
    node_=node;plugin_name_="RPP";min_approach_linear_velocity_=.05;
  }
  double point(const nav_msgs::msg::Path & p) {
    syncAdditionalParameters();return getLookAheadPoint(1.,p).pose.position.x;
  }
  double approach(const nav_msgs::msg::Path & p) {
    syncAdditionalParameters();double v=.4;applyApproachVelocityScaling(p,v);return v;
  }
};
TEST(HotRpp, ActualInterpolationAndApproachChange) {
  rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("rpp_probe");
  node->declare_parameter("RPP.use_interpolation",false);
  node->declare_parameter("RPP.approach_velocity_scaling_dist",1.);
  RppProbe p;p.setup(node);
  nav_msgs::msg::Path path;path.poses.resize(2);path.poses.back().pose.position.x=2.;
  EXPECT_DOUBLE_EQ(p.point(path),2.);
  node->set_parameter(rclcpp::Parameter("RPP.use_interpolation",true));
  EXPECT_NEAR(p.point(path),1.,1e-6);
  path.poses.back().pose.position.x=.5;
  EXPECT_NEAR(p.approach(path),.2,1e-6);
  node->set_parameter(rclcpp::Parameter("RPP.approach_velocity_scaling_dist",2.));
  EXPECT_NEAR(p.approach(path),.1,1e-6);
  rclcpp::shutdown();
}
