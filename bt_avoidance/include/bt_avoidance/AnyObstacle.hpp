#ifndef _ANYOBSTACLE_HPP_
#define _ANYOBSTACLE_HPP_

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_avoidance
{
class AnyObstacle : public BT::ConditionNode
{
public:
  explicit AnyObstacle(
    const std::string& xml_tag_name,
    const BT::NodeConfiguration& conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({BT::InputPort<double>("minDist")});
  }

  void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_reading_time_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

} // namespace bt_avoidance

#endif /* _ANYOBSTACLE_HPP_ */
