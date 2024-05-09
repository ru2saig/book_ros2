#include "bt_avoidance/MoveAway.hpp"
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree_node.h>
#include <cmath>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <vector>

namespace bt_avoidance
{
MoveAway::MoveAway(
  const std::string& xml_tag_name,
  const BT::NodeConfiguration& conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 1);
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>("/input_scan", 1, std::bind(&MoveAway::laser_callback, this, std::placeholders::_1));
}

void
MoveAway::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

BT::NodeStatus
MoveAway::tick()
{
  if (last_scan_ == nullptr) return BT::NodeStatus::FAILURE;

  double minDist = 1.5;
  getInput("minDist", minDist);

  std::array<double, 2> attractive = { 1.0, 0.0 };
  std::array<double, 2> repulsive = { 0.0, 0.0 };
  std::array<double, 2> result = { 0.0, 0.0 };

  int obstacles = 0;
  
  for (std::vector<double>::size_type i = 0; i < last_scan_->ranges.size(); i++) 
    if (last_scan_->ranges[i] < minDist && last_scan_->ranges[i] > last_scan_->range_min) {
      obstacles++;
       
      float angle = M_PI + last_scan_->angle_min + last_scan_->angle_increment * i;
       
      // The module of the vector is inverse to the distance to the obstacle
      float mag = minDist - last_scan_->ranges[i];
  
      // Get cartesian (x, y) components from polar (angle, distance)
      repulsive[0] += cos(angle) * mag;
      repulsive[1] += sin(angle) * mag;
    }
  
  if (obstacles) {
    repulsive[0] = (repulsive[0]/obstacles);
    repulsive[1] = (repulsive[1]/obstacles);

    // normalizing
    double mag = std::sqrt(repulsive[1]*repulsive[1] + repulsive[0]*repulsive[0]);
    repulsive[0] = repulsive[0]/mag;
    repulsive[1] = repulsive[1]/mag;
  }

  result[0] = (repulsive[0] + attractive[0]);
  result[1] = (repulsive[1] + attractive[1]);

  double angle = std::atan2(result[1], result[0]);
  double mag = std::sqrt(result[1]*result[1] + result[0]*result[0]);

  geometry_msgs::msg::Twist vel;
  vel.linear.x = std::clamp(mag, 0.0, 0.3);  // truncate linear vel to [0.0, 0.3] m/s
  vel.angular.z = std::clamp(angle, -0.5, 0.5);  // truncate rotation vel to [-0.5, 0.5] rad/s

  vel_pub_->publish(vel);

  return BT::NodeStatus::RUNNING;
}

} // namespace bt_avoidance

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_avoidance::MoveAway>("MoveAway");
}
