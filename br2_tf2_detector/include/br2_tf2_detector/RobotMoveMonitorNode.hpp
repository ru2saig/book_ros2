#ifndef BR2_TF2_DETECTOR__ROBOTMOVEMONITORNODE_HPP__
#define BR2_TF2_DETECTOR__ROBOTMOVEMONITORNODE_HPP__

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


#include <rclcpp/rclcpp.hpp>

namespace br2_tf2_detector
{

class RobotMoveMonitorNode : public rclcpp::Node
{
public:
    RobotMoveMonitorNode();

private:
    void control_cycle();
    rclcpp::TimerBase::SharedPtr timer_;

    tf2::BufferCore tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::TransformStamped last_odom2robot; // store the last seen position of the robot!

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

} // namespace br2_tf2_detector

#endif