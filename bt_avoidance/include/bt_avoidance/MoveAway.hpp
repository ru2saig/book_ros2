#ifndef _MOVEAWAY_HPP_
#define _MOVEAWAY_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/basic_types.h>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>

namespace bt_avoidance
{
class MoveAway : public BT::ActionNodeBase
{
public:
  explicit MoveAway(
    const std::string& xml_tag_name,
    const BT::NodeConfiguration& conf);

  void halt() {}
	
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({BT::InputPort<double>("minDist")});
  }

  void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  
private:
  rclcpp::Node::SharedPtr node_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

} // namespace bt_avoidance

#endif /* _MOVEAWAY_HPP_ */
