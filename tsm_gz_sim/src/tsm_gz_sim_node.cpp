#include "gz_sim/tsm_gz_sim_node.hpp"

#include "gz/msgs/boolean.pb.h"
#include "gz/msgs/pose.pb.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace tsm
{

TsmGzSimNode::TsmGzSimNode(const rclcpp::NodeOptions& options)
    : Node("tsm_gz_sim_node", options), tf_broadcaster_(this)
{
  int    timer_ms = declare_parameter<int>("control_timer_ms", 10);
  double tube_length = declare_parameter<double>("tube_length", 10.0);
  speed_ = declare_parameter<double>("speed", 0.1);
  set_pose_service_ = declare_parameter<std::string>(
    "set_pose_service", "/world/measure_world/set_pose");
  tube_model_name_ = declare_parameter<std::string>("tube_model_name", "straight_tube");
  tf_frame_id_ = declare_parameter<std::string>("tf_frame_id", "world");
  tf_child_frame_id_ = declare_parameter<std::string>("tf_child_frame_id", "tube");

  dt_s_ = timer_ms / 1000.0;
  x_start_ = -tube_length / 2.0;
  x_end_ = tube_length / 2.0;
  x_pos_ = x_start_;

  cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "tube_cmd",
      10,
      std::bind(&TsmGzSimNode::onCmd, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(this,
                                get_clock(),
                                std::chrono::milliseconds(timer_ms),
                                std::bind(&TsmGzSimNode::onTimer, this));
}

void TsmGzSimNode::onCmd(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "f")
    direction_ = 1;
  else if (msg->data == "b")
    direction_ = -1;
  else if (msg->data == "s")
    direction_ = 0;
}

void TsmGzSimNode::onTimer()
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = get_clock()->now();
  tf.header.frame_id = tf_frame_id_;
  tf.child_frame_id = tf_child_frame_id_;
  tf.transform.translation.x = x_pos_;
  tf.transform.rotation.w = 1.0;
  tf_broadcaster_.sendTransform(tf);

  if (direction_ == 0)
    return;

  x_pos_ += direction_ * speed_ * dt_s_;
  if (x_pos_ >= x_end_)
  {
    x_pos_ = x_end_;
    direction_ = 0;
  }
  else if (x_pos_ <= x_start_)
  {
    x_pos_ = x_start_;
    direction_ = 0;
  }

  gz::msgs::Pose req;
  req.set_name(tube_model_name_);
  req.mutable_position()->set_x(x_pos_);
  req.mutable_orientation()->set_w(1.0);

  gz::msgs::Boolean rep;
  bool              result = false;
  gz_node_.Request(set_pose_service_, req, 500, rep, result);
}

} // namespace tsm

RCLCPP_COMPONENTS_REGISTER_NODE(tsm::TsmGzSimNode)
