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

#include "pcl/common/common.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types_conversion.h"
#include "pcl/filters/extract_indices.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer_interface.h"
#include "tf2/transform_datatypes.h"
#include "pcl_ros/transforms.hpp"
#include "visualization_msgs/msg/detail/marker__struct.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"

#include "br2_tracking/ObjectDetector.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <geometry_msgs/msg/detail/point__struct.hpp>

namespace br2_tracking
{

using std::placeholders::_1;
using namespace std::chrono_literals;
    
ObjectDetector::ObjectDetector()
    : Node("object_detector"),
      tf_buffer(),
      tf_listener(tf_buffer)
{
  image_sub_ = image_transport::create_subscription(
    this, "input_image", std::bind(&ObjectDetector::image_callback, this, _1),
    "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("point_cloud", 100, std::bind(&ObjectDetector::pointcloud_callback, this, _1));

  detection_pub_ = create_publisher<vision_msgs::msg::Detection2D>("detection", 100);
  box_pub_ = create_publisher<visualization_msgs::msg::Marker>("obstacle_bb", 1);
  
  declare_parameter("hsv_ranges", hsv_filter_ranges_);
  declare_parameter("debug", debug_);

  get_parameter("hsv_ranges", hsv_filter_ranges_);
  get_parameter("debug", debug_);

}

bool
ObjectDetector::inRange(pcl::PointXYZHSV &p)
{
    return ((hsv_filter_ranges_[0] < p.h && p.h < hsv_filter_ranges_[1]) &&
	    (hsv_filter_ranges_[2] < (p.s * 255) && (p.s * 255) < hsv_filter_ranges_[3]) &&
	    (hsv_filter_ranges_[4] < (p.v * 255) && (p.v * 255) < hsv_filter_ranges_[5]));
}

// TODO: Takes a good 5s for it to get up to speed with the latest point cloud. Probably the filter?
void
ObjectDetector::pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  if (box_pub_->get_subscription_count() == 0) {return;}
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr pointsHSV = std::make_shared<pcl::PointCloud<pcl::PointXYZHSV>>();
  
  pcl::fromROSMsg(*msg, *points);

  geometry_msgs::msg::TransformStamped odom2cam_msg;
  tf2::Stamped<tf2::Transform> odom2cam_tf;

  // Obtaining the transformation from odom -> depth camera (hence, get where the point cloud is in odom)
  try {
    odom2cam_msg = tf_buffer.lookupTransform("odom", msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp));
    tf2::fromMsg(odom2cam_msg, odom2cam_tf);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(get_logger(), "Obstacle transform not found: " << ex.what());
    return;
  }

  // The actual transformation; applied on points as msg has a const qualifier
  pcl_ros::transformPointCloud(*points, *points, odom2cam_tf);

  // Finding the points belonging to any obstacle, using HSV thresholding
  pcl::PointCloudXYZRGBtoXYZHSV(*points, *pointsHSV);
  pcl::PointIndices::Ptr obstacle(new pcl::PointIndices());

  for (int i = 0; i < pointsHSV->size(); i++) {
      if(inRange(pointsHSV->points[i]))
	  obstacle->indices.push_back(i);
  }

  if (obstacle->indices.size() < 20)
      return;

  // Finding the AABB
  // pcl::PointXYZHSV centroid;
  // pcl::computeCentroid(*pointsHSV, obstacle->indices, centroid);

  Eigen::Vector4f minPoint;
  Eigen::Vector4f maxPoint;
  
  pcl::getMinMax3D(*pointsHSV, obstacle->indices, minPoint, maxPoint);

  // The vertices of the bounding box ;-;
  std::vector<geometry_msgs::msg::Point> bbPoints;
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  geometry_msgs::msg::Point p2;
  geometry_msgs::msg::Point p3;
  geometry_msgs::msg::Point p4;
  geometry_msgs::msg::Point p5;
  geometry_msgs::msg::Point p6;
  geometry_msgs::msg::Point p7;

  auto xm = minPoint.x();
  auto xM = maxPoint.x();
  auto ym = minPoint.y();
  auto yM = maxPoint.y();
  auto zm = minPoint.z();
  auto zM = maxPoint.z();
  
  p0.x = xm;
  p0.y = ym;
  p0.z = zm;

  p1.x = xm;
  p1.y = ym;
  p1.z = zM;

  p2.x = xm;
  p2.y = yM;
  p2.z = zM;
  
  p3.x = xm;
  p3.y = yM;
  p3.z = zm;
  
  p4.x = xM;
  p4.y = yM;
  p4.z = zm;
  
  p5.x = xM;
  p5.y = yM;
  p5.z = zM;
  
  p6.x = xM;
  p6.y = ym;
  p6.z = zM;
  
  p7.x = xM;
  p7.y = ym;
  p7.z = zm;
    
  bbPoints.emplace_back(p0);
  bbPoints.emplace_back(p1);
  bbPoints.emplace_back(p0);
  bbPoints.emplace_back(p3);
  bbPoints.emplace_back(p0);
  bbPoints.emplace_back(p7);
  bbPoints.emplace_back(p1);
  bbPoints.emplace_back(p6);
  bbPoints.emplace_back(p1);
  bbPoints.emplace_back(p2);
  bbPoints.emplace_back(p2);
  bbPoints.emplace_back(p3);
  bbPoints.emplace_back(p2);
  bbPoints.emplace_back(p5);
  bbPoints.emplace_back(p3);
  bbPoints.emplace_back(p4);
  bbPoints.emplace_back(p4);
  bbPoints.emplace_back(p5);
  bbPoints.emplace_back(p4);
  bbPoints.emplace_back(p7);
  bbPoints.emplace_back(p5);
  bbPoints.emplace_back(p6);
  bbPoints.emplace_back(p6);
  bbPoints.emplace_back(p7);
    
  visualization_msgs::msg::Marker box;
  box.header.frame_id = "odom";
  box.header.stamp = msg->header.stamp;
  box.ns = "bounding_box";
  box.id = 0;
  box.type = visualization_msgs::msg::Marker::LINE_LIST;
  box.action = visualization_msgs::msg::Marker::ADD;
  box.points = bbPoints;
  
  box.pose.position.x = 0.0;
  box.pose.position.y = 0.0;
  box.pose.position.z = 0.0;
  box.pose.orientation.x = 0.0;
  box.pose.orientation.y = 0.0;
  box.pose.orientation.z = 0.0;
  box.pose.orientation.w = 1.0;

  box.scale.x = 0.01;

  box.color.r = 0.0f;
  box.color.g = 1.0f;
  box.color.b = 0.0f;
  box.color.a = 1.0;

  box.lifetime = rclcpp::Duration(1s);
  
  box_pub_->publish(box);
}

// TODO: Remove this, in favour of pointcloud_callback. Detection2D is a raycast transformed bounding box centroid?
void
ObjectDetector::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (detection_pub_->get_subscription_count() == 0) {return;}

  const float & h = hsv_filter_ranges_[0];
  const float & H = hsv_filter_ranges_[1];
  const float & s = hsv_filter_ranges_[2];
  const float & S = hsv_filter_ranges_[3];
  const float & v = hsv_filter_ranges_[4];
  const float & V = hsv_filter_ranges_[5];

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img_hsv;
  cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);

  cv::Mat1b filtered;
  cv::inRange(img_hsv, cv::Scalar(h, s, v), cv::Scalar(H, S, V), filtered);

  auto moment = cv::moments(filtered, true);
  cv::Rect bbx = cv::boundingRect(filtered);

  auto m = cv::moments(filtered, true);
  if (m.m00 < 0.000001) {return;}
  int cx = m.m10 / m.m00;
  int cy = m.m01 / m.m00;

  vision_msgs::msg::Detection2D detection_msg;
  detection_msg.header = msg->header;
  detection_msg.bbox.size_x = bbx.width;
  detection_msg.bbox.size_y = bbx.height;
  detection_msg.bbox.center.position.x = cx;
  detection_msg.bbox.center.position.y = cy;
  detection_pub_->publish(detection_msg);

  if (debug_) {
    cv::rectangle(cv_ptr->image, bbx, cv::Scalar(0, 0, 255), 3);
    cv::circle(cv_ptr->image, cv::Point(cx, cy), 3, cv::Scalar(255, 0, 0), 3);
    cv::imshow("cv_ptr->image", cv_ptr->image);
    cv::waitKey(1);
  }
}

}  // namespace br2_tracking
