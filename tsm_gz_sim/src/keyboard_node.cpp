///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

#include "gz_sim/keyboard_node.hpp"

#include <termios.h>
#include <unistd.h>

#include "rclcpp_components/register_node_macro.hpp"

namespace tsm
{

KeyboardNode::KeyboardNode(const rclcpp::NodeOptions & options)
: Node("keyboard_node", options)
{
  cmd_pub_ = create_publisher<std_msgs::msg::String>("tube_cmd", 10);
  RCLCPP_INFO(get_logger(), "f=forward  b=backward  s=stop  q=quit");
  read_thread_ = std::thread(&KeyboardNode::readLoop, this);
}

KeyboardNode::~KeyboardNode()
{
  running_ = false;
  if (read_thread_.joinable()) read_thread_.join();
}

void KeyboardNode::readLoop()
{
  struct termios t_old, t_new;
  tcgetattr(STDIN_FILENO, &t_old);
  t_new = t_old;
  t_new.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &t_new);

  while (running_) {
    char c = getchar();
    if (c == 'q') { running_ = false; break; }
    if (c != 'f' && c != 'b' && c != 's') continue;
    std_msgs::msg::String msg;
    msg.data = std::string(1, c);
    cmd_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "cmd: %c", c);
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &t_old);
}

} // namespace tsm

RCLCPP_COMPONENTS_REGISTER_NODE(tsm::KeyboardNode)
