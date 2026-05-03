///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace tsm
{

class KeyboardNode : public rclcpp::Node
{
public:
  explicit KeyboardNode(const rclcpp::NodeOptions & options);
  ~KeyboardNode();

private:
  void readLoop();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  std::thread read_thread_;
  std::atomic<bool> running_{true};
};

} // namespace tsm
