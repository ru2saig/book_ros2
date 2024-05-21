// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BR2_TRACKING__OBJECTDETECTOR_HPP_
#define BR2_TRACKING__OBJECTDETECTOR_HPP_

#include <pcl/impl/point_types.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include "pcl/point_cloud.h"
#include <vector>
#include <vision_msgs/msg/detail/bounding_box3_d__struct.hpp>
#include <Eigen/Eigenvalues> 
#include <visualization_msgs/msg/detail/marker__struct.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_tracking
{

class ObjectDetector : public rclcpp::Node
{
public:
  ObjectDetector();

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

private:
  tf2::BufferCore tf_buffer;
  tf2_ros::TransformListener tf_listener;
    
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr detection_pub_;
  // TODO: Figure out how to display this; could I use a Marker? Perhaps a line strip?
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr box_pub_;

  // HSV ranges for detection [h - H] [s - S] [v - V]
  std::vector<double> hsv_filter_ranges_ {0, 180, 0, 255, 0, 255};
  bool inRange(pcl::PointXYZHSV &p);
  
  bool debug_ {true};
};

}  // namespace br2_tracking

#endif  // BR2_TRACKING__OBJECTDETECTOR_HPP_
