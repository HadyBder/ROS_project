#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace br2_bt_patrolling
{

class MoveToFreeSpace : public BT::SyncActionNode
{
public:
  MoveToFreeSpace(const std::string &name, const BT::NodeConfiguration &config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() { return {}; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

}  
