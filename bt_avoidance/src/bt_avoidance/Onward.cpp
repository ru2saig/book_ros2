#include "bt_avoidance/Onward.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <behaviortree_cpp_v3/basic_types.h>
#include <string>

namespace bt_avoidance
{
Onward::Onward(
  const std::string& xml_tag_name,
  const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);
}

BT::NodeStatus
Onward::tick()
{
  geometry_msgs::msg::Twist vel;
  vel.angular.z = 0.0;
  vel.linear.x = 0.3;
  vel_pub_->publish(vel);

  return BT::NodeStatus::RUNNING;
}

} // namespace bt_avoidance


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_avoidance::Onward>("Onward");
}
