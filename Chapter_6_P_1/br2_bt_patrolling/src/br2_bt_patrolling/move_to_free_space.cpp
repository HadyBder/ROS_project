#include "br2_bt_patrolling/move_to_free_space.hpp"

using namespace br2_bt_patrolling;

MoveToFreeSpace::MoveToFreeSpace(const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("move_to_free_space_node");

  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_raw", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      latest_scan_ = msg;
    });

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/key_vel", 10);
}

BT::NodeStatus MoveToFreeSpace::tick()
{
  if (!latest_scan_) return BT::NodeStatus::FAILURE;

  float max_dist = 0.0;
  int best_index = -1;

  for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
    float r = latest_scan_->ranges[i];
    if (std::isfinite(r) && r > max_dist) {
      max_dist = r;
      best_index = i;
    }
  }

  if (best_index == -1) return BT::NodeStatus::FAILURE;

  float angle = latest_scan_->angle_min + best_index * latest_scan_->angle_increment;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.2;
  cmd.angular.z = angle;
  cmd_pub_->publish(cmd);

  return BT::NodeStatus::SUCCESS;
}
