#include "bt_avoidance/AnyObstacle.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/tree_node.h>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>

namespace bt_avoidance
{
AnyObstacle::AnyObstacle(
  const std::string& xml_tag_name,
  const BT::NodeConfiguration& conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>("/input_scan", 1, std::bind(&AnyObstacle::laser_callback, this, std::placeholders::_1));

  last_reading_time_ = node_->now();
}

void
AnyObstacle::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

BT::NodeStatus
AnyObstacle::tick()
{
  if (last_scan_ == nullptr) return BT::NodeStatus::FAILURE;

  double minDist = 1.0;
  double rangeMin = last_scan_->range_min;
  getInput("minDist", minDist);

  if (std::any_of(last_scan_->ranges.begin(), last_scan_->ranges.end(), [minDist, rangeMin] (double reading) { return reading < minDist && reading > rangeMin; })) {
    last_scan_ = nullptr;
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace bt_avoidance

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_avoidance::AnyObstacle>("AnyObstacle");
}

