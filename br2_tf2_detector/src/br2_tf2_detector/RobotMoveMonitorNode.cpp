#include <functional>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>

#include "br2_tf2_detector/RobotMoveMonitorNode.hpp"

namespace br2_tf2_detector
{

using namespace std::chrono_literals;

RobotMoveMonitorNode::RobotMoveMonitorNode()
: Node("robot_mov_monitor"),
tf_buffer_(),
tf_listener_(tf_buffer_)
{
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("robot_mov_marker", 1);

    timer_ = create_wall_timer(
        1s, std::bind(&RobotMoveMonitorNode::control_cycle, this));
}

void RobotMoveMonitorNode::control_cycle()
{
    geometry_msgs::msg::TransformStamped odom2robot;

    try {
        // why not lookupTransform 1 second back in time?
        // Runs 1 second anyhow, no need of the overhead?
        odom2robot = tf_buffer_.lookupTransform(
            "odom", "base_footprint", tf2::TimePointZero);        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "Robot transform not found :[ %s", ex.what());
        return;
    }

    double x = odom2robot.transform.translation.x;
    double y = odom2robot.transform.translation.y;
    double z = odom2robot.transform.translation.z;
    double theta = atan2(y, x);

    // RCLCPP_INFO(get_logger(), "Robot was at (%lf m, %lf m, %lf m) = %lf rads", last_odom2robot.transform.translation.x, last_odom2robot.transform.translation.y, last_odom2robot.transform.translation.z, theta);
    // RCLCPP_INFO(get_logger(), "Robot is at (%lf m, %lf m, %lf m) = %lf rads", x, y, z, theta);

    visualization_msgs::msg::Marker robot_arrow;
    robot_arrow.header.frame_id = "odom";
    robot_arrow.header.stamp = now();
    robot_arrow.type = visualization_msgs::msg::Marker::ARROW;
    robot_arrow.action = visualization_msgs::msg::Marker::ADD;
    robot_arrow.lifetime = rclcpp::Duration(1s);

    geometry_msgs::msg::Point start;
    start.x = last_odom2robot.transform.translation.x;
    start.y = last_odom2robot.transform.translation.y;
    start.z = last_odom2robot.transform.translation.z;

    geometry_msgs::msg::Point end;
    end.x = x;
    end.y = y;
    end.z = z;

    robot_arrow.points = {start, end};ll

    robot_arrow.color.r = 0.0;
    robot_arrow.color.g = 1.0;
    robot_arrow.color.b = 0.0;
    robot_arrow.color.a = 1.0;

    robot_arrow.scale.x = 0.02;
    robot_arrow.scale.y = 0.1;
    robot_arrow.scale.z = 0.1;

    marker_pub_->publish(robot_arrow);
    last_odom2robot = odom2robot;
}


}