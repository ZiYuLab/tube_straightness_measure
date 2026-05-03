///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

#pragma once

#include "rclcpp/rclcpp.hpp"



namespace tsm
{

class TsmNode : public rclcpp::Node
{
public:
  explicit TsmNode(const rclcpp::NodeOptions & options);

private:
  void setupParams();
  void setupPub();
  void setupSub();
};

} // namespace tsm
