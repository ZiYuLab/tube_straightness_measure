///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Description:
// Resenting and processing some msgs from the simulation environment.
// And control the move of the tube and add some noise on it.

#pragma once

#include <gz/transport/Node.hh>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace tsm
{

class TsmGzSimNode : public rclcpp::Node
{
public:
  explicit TsmGzSimNode(const rclcpp::NodeOptions & options);

private:
  void onTimer();
  void onCmd(const std_msgs::msg::String::SharedPtr msg);

  gz::transport::Node gz_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  double x_pos_{0.0};
  double speed_{0.1};
  double x_end_{0.5};
  double x_start_{-0.5};
  double dt_s_{0.001};
  double vib_amplitude_{0.002};
  double vib_frequency_{1.0};
  double tilt_angle_{0.0};       // static tilt around z-axis (rad)
  double tilt_vib_amplitude_{0.0};  // angular vibration amplitude (rad)
  double tilt_vib_frequency_{0.5};  // angular vibration frequency (Hz)
  double elapsed_s_{0.0};
  int direction_{0};
  std::string tf_frame_id_{"world"};
  std::string tf_child_frame_id_{"tube"};
  std::string set_pose_service_{"/world/measure_world/set_pose"};
  std::string tube_model_name_{"straight_tube"};
};

} // namespace tsm
