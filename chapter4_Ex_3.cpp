// === monitor_node_ex3.cpp ===
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
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
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&MonitorNode::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float range = msg->ranges[i];
      if (!std::isfinite(range) || range <= 0.05f) continue;

      float angle = msg->angle_min + i * msg->angle_increment;

      geometry_msgs::msg::PointStamped p_laser, p_odom;
      p_laser.header = msg->header;
      p_laser.point.x = range * cos(angle);
      p_laser.point.y = range * sin(angle);
      p_laser.point.z = 0.0;

      try {
        tf_buffer_.transform(p_laser, p_odom, "odom", tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "odom";
      marker.header.stamp = this->now();
      marker.ns = "obstacles";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = p_odom.point;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.15;
      marker.scale.y = 0.15;
      marker.scale.z = 0.15;
      marker.color.a = 1.0;

      if (range > 1.0f) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      } else {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
      }

      markers.markers.push_back(marker);
    }

    marker_pub_->publish(markers);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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
}
