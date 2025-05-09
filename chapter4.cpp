// === monitor_node_ex1.cpp ===
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode() : Node("monitor_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&MonitorNode::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Find the closest valid range
    float min_dist = std::numeric_limits<float>::infinity();
    int min_index = -1;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < min_dist)
      {
        min_dist = msg->ranges[i];
        min_index = static_cast<int>(i);
      }
    }

    if (min_index == -1)
      return;  // No valid obstacle

    // Calculate the obstacle position in base_laser_link frame
    float angle = msg->angle_min + min_index * msg->angle_increment;
    geometry_msgs::msg::PointStamped obstacle_in_laser;
    obstacle_in_laser.header.frame_id = msg->header.frame_id;  // typically "base_laser_link"
    obstacle_in_laser.header.stamp = msg->header.stamp;
    obstacle_in_laser.point.x = min_dist * cos(angle);
    obstacle_in_laser.point.y = min_dist * sin(angle);
    obstacle_in_laser.point.z = 0.0;

    // Transform to /odom frame
    geometry_msgs::msg::PointStamped obstacle_in_odom;
    try
    {
      tf_buffer_.transform(obstacle_in_laser, obstacle_in_odom, "odom", tf2::durationFromSec(0.2));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform to odom: %s", ex.what());
      return;
    }

    // Publish as a marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "obstacle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = obstacle_in_odom.point.x;
    marker.pose.position.y = obstacle_in_odom.point.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker_pub_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNode>());
  rclcpp::shutdown();
  return 0;
} // === END ===
