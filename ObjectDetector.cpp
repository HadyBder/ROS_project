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

#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <vision_msgs/msg/detection2_d.hpp>

#include "br2_tracking/ObjectDetector.hpp"

namespace br2_tracking
{

using std::placeholders::_1;

ObjectDetector::ObjectDetector()
: Node("object_detector")
{
  image_sub_ = image_transport::create_subscription(
    this, "input_image", std::bind(&ObjectDetector::image_callback, this, _1),
    "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  detection_pub_ = create_publisher<vision_msgs::msg::Detection2D>("detection", 100);

  // Declare and read HSV range and debug parameters
  declare_parameter("hsv_ranges", hsv_filter_ranges_);
  declare_parameter("debug", debug_);
  get_parameter("hsv_ranges", hsv_filter_ranges_);
  get_parameter("debug", debug_);
}

void ObjectDetector::image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Only publish if someone is listening
  if (detection_pub_->get_subscription_count() == 0) {
    return;
  }

  // Unpack HSV filter ranges
  const float & h = hsv_filter_ranges_[0];
  const float & H = hsv_filter_ranges_[1];
  const float & s = hsv_filter_ranges_[2];
  const float & S = hsv_filter_ranges_[3];
  const float & v = hsv_filter_ranges_[4];
  const float & V = hsv_filter_ranges_[5];

  // Convert ROS image to OpenCV BGR
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Convert to HSV and apply threshold
  cv::Mat img_hsv;
  cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);
  cv::Mat1b filtered;
  cv::inRange(img_hsv, cv::Scalar(h, s, v), cv::Scalar(H, S, V), filtered);

  // find contours for each object
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(filtered, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    return;
  }

  // Compute bounding box for each contour
  std::vector<cv::Rect> boxes;
  boxes.reserve(contours.size());
  for (const auto & cnt : contours) {
    boxes.emplace_back(cv::boundingRect(cnt));
  }

  // Select the most recently detected object (last in list)
  cv::Rect bbx = boxes.back();
  int cx = bbx.x + bbx.width / 2;
  int cy = bbx.y + bbx.height / 2;

  // Publish Detection2D message
  vision_msgs::msg::Detection2D detection_msg;
  detection_msg.header = msg->header;
  detection_msg.bbox.size_x   = bbx.width;
  detection_msg.bbox.size_y   = bbx.height;
  detection_msg.bbox.center.x = cx;
  detection_msg.bbox.center.y = cy;
  detection_msg.source_img    = *cv_ptr->toImageMsg();
  detection_pub_->publish(detection_msg);

  //draw the chosen box and center
  if (debug_) {
    cv::rectangle(cv_ptr->image, bbx, cv::Scalar(0, 0, 255), 3);
    cv::circle(cv_ptr->image, cv::Point(cx, cy), 3, cv::Scalar(255, 0, 0), 3);
    cv::imshow("ObjectDetector Debug", cv_ptr->image);
    cv::waitKey(1);
  }
}

}  // namespace br2_tracking
