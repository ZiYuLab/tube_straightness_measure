#include "gz_sim/tsm_gz_sim_node.hpp"

#include "gz/msgs/boolean.pb.h"
#include "gz/msgs/pose.pb.h"

#include "rclcpp_components/register_node_macro.hpp"

namespace tsm
{

TsmGzSimNode::TsmGzSimNode(const rclcpp::NodeOptions & options)
: Node("tsm_gz_sim_node", options)
{
  declare_parameter<std::string>("set_pose_topic", "/set_pose");
  declare_parameter<std::string>("tube_pose_topic", "/tube_pose");
  int timer_ms = declare_parameter<int>("control_timer_ms", 10);
  double tube_length = declare_parameter<double>("tube_length", 10.0);
  speed_ = declare_parameter<double>("speed", 0.1);

  dt_s_ = timer_ms / 1000.0;
  x_start_ = -tube_length / 2.0;
  x_end_ = tube_length / 2.0;
  x_pos_ = x_start_;

  cmd_sub_ = create_subscription<std_msgs::msg::String>(
    "tube_cmd", 10, std::bind(&TsmGzSimNode::onCmd, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, get_clock(),
    std::chrono::milliseconds(timer_ms),
    std::bind(&TsmGzSimNode::onTimer, this));
}

void TsmGzSimNode::onCmd(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "f") direction_ = 1;
  else if (msg->data == "b") direction_ = -1;
  else if (msg->data == "s") direction_ = 0;
}

void TsmGzSimNode::onTimer()
{
  if (direction_ == 0) return;

  x_pos_ += direction_ * speed_ * dt_s_;
  if (x_pos_ >= x_end_) { x_pos_ = x_end_; direction_ = 0; }
  else if (x_pos_ <= x_start_) { x_pos_ = x_start_; direction_ = 0; }

  gz::msgs::Pose req;
  req.set_name("straight_tube");
  req.mutable_position()->set_x(x_pos_);
  req.mutable_orientation()->set_w(1.0);

  gz::msgs::Boolean rep;
  bool result = false;
  gz_node_.Request("/world/measure_world/set_pose", req, 500, rep, result);
}

} // namespace tsm

RCLCPP_COMPONENTS_REGISTER_NODE(tsm::TsmGzSimNode)
