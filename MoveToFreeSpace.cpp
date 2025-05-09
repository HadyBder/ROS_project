#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

class MoveToFreeSpace : public BT::SyncActionNode {
public:
  MoveToFreeSpace(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config), obstacle_detected_(false)
  {
    node_ = rclcpp::Node::make_shared("move_to_free_space_node");

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_raw", rclcpp::SensorDataQoS(),
      std::bind(&MoveToFreeSpace::scan_callback, this, std::placeholders::_1));

    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10);
  }

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override {
    rclcpp::spin_some(node_);

    geometry_msgs::msg::Twist cmd;
    if (!obstacle_detected_) {
      cmd.linear.x = 0.2;
      vel_pub_->publish(cmd);
      return BT::NodeStatus::RUNNING;
    } else {
      cmd.linear.x = 0.0;
      vel_pub_->publish(cmd);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    obstacle_detected_ = (min_range < 0.6); // Threshold
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  bool obstacle_detected_;
};
