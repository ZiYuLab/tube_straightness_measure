///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

#include "tsm/tsm_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace tsm
{

TsmNode::TsmNode(const rclcpp::NodeOptions & options)
: Node("tsm_node", options)
{
}

void TsmNode::setupParams()
{
}

void TsmNode::setupPub()
{
}

void TsmNode::setupSub()
{
}

} // namespace tsm

RCLCPP_COMPONENTS_REGISTER_NODE(tsm::TsmNode)
