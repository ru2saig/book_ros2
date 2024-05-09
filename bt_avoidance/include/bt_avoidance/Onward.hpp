#ifndef _ONWARD_HPP_
#define _ONWARD_HPP_

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_avoidance
{
class Onward : public BT::ActionNodeBase
{
public:
  explicit Onward(
    const std::string& xml_tag_name,
    const BT::NodeConfiguration& conf);

  void halt() {}
	
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
      return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

} // namespace bt_avoidance

#endif /* _ONWARD_HPP_ */
