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

#include <memory>
#include <utility>
#include <algorithm>
#include <vector>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "br2_vff_avoidance/AvoidanceNode.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace br2_vff_avoidance
{

AvoidanceNode::AvoidanceNode()
: Node("avoidance_vff")
{
  // Declare tunable parameters
  this->declare_parameter<double>("max_linear_speed", 0.3);
  this->declare_parameter<double>("repulsive_gain", 1.0);
  this->declare_parameter<double>("safety_distance", 0.2);

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
  vff_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 100);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(), std::bind(&AvoidanceNode::scan_callback, this, _1));

  timer_ = create_wall_timer(50ms, std::bind(&AvoidanceNode::control_cycle, this));
}

void AvoidanceNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void AvoidanceNode::control_cycle()
{
  // Skip if no valid scan
  if (last_scan_ == nullptr || (now() - last_scan_->header.stamp) > 1s) {
    return;
  }

  const auto & scan = *last_scan_;
  const VFFVectors & vff = get_vff(scan);
  const auto & v = vff.result;

  // Retrieve parameters
  double max_lin     = this->get_parameter("max_linear_speed").as_double();
  double rep_gain    = this->get_parameter("repulsive_gain").as_double();
  double safety_dist = this->get_parameter("safety_distance").as_double();

  // Compute emergency stop condition
  float min_range = *std::min_element(scan.ranges.begin(), scan.ranges.end());

  geometry_msgs::msg::Twist vel;

  if (min_range < safety_dist) {
    // Emergency stop: zero velocities
    vel.linear.x  = 0.0;
    vel.angular.z = 0.0;
  } else {
    // Normal VFF-based control with proper type for clamp
    vel.linear.x  = std::clamp(static_cast<double>(v[0]), 0.0, max_lin);  // <--- cast v[0] to double
    vel.angular.z = std::clamp(v[1] * rep_gain, -0.5, 0.5);
  }

  vel_pub_->publish(vel);

  if (vff_debug_pub_->get_subscription_count() > 0) {
    vff_debug_pub_->publish(get_debug_vff(vff));
  }
}

VFFVectors AvoidanceNode::get_vff(const sensor_msgs::msg::LaserScan & scan)
{
  const float OBSTACLE_DISTANCE = 1.0;

  VFFVectors vff_vector;
  vff_vector.attractive = {OBSTACLE_DISTANCE, 0.0};
  vff_vector.repulsive = {0.0, 0.0};
  vff_vector.result    = {0.0, 0.0};

  // Accumulate repulsion from all beams
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    if (std::isfinite(r) && r < OBSTACLE_DISTANCE) {
      float theta     = scan.angle_min + scan.angle_increment * i;
      float strength  = OBSTACLE_DISTANCE - r;
      float opp_theta = theta + M_PI;
      vff_vector.repulsive[0] += std::cos(opp_theta) * strength;
      vff_vector.repulsive[1] += std::sin(opp_theta) * strength;
    }
  }

  // Sum vectors
  vff_vector.result[0] = vff_vector.attractive[0] + vff_vector.repulsive[0];
  vff_vector.result[1] = vff_vector.attractive[1] + vff_vector.repulsive[1];

  return vff_vector;
}

visualization_msgs::msg::MarkerArray AvoidanceNode::get_debug_vff(
  const VFFVectors & vff_vectors)
{
  visualization_msgs::msg::MarkerArray array;
  array.markers.push_back(make_marker(vff_vectors.attractive, BLUE));
  array.markers.push_back(make_marker(vff_vectors.repulsive, RED));
  array.markers.push_back(make_marker(vff_vectors.result, GREEN));
  return array;
}

visualization_msgs::msg::Marker AvoidanceNode::make_marker(
  const std::vector<float> & vec,
  VFFColor vff_color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_footprint";
  marker.header.stamp    = now();
  marker.type            = visualization_msgs::msg::Marker::ARROW;
  marker.scale.x         = 0.05;
  marker.scale.y         = 0.1;

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  geometry_msgs::msg::Point end;
  end.x = vec[0];
  end.y = vec[1];
  marker.points = {start, end};

  switch (vff_color) {
    case RED:
      marker.id    = 0;
      marker.color.r = 1.0;
      break;
    case GREEN:
      marker.id    = 1;
      marker.color.g = 1.0;
      break;
    case BLUE:
      marker.id    = 2;
      marker.color.b = 1.0;
      break;
  }
  marker.color.a = 1.0;

  return marker;
}

}  // namespace br2_vff_avoidance
