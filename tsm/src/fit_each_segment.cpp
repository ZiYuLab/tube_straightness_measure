///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// This file is used to fit each segment of the tube to find the circle center.
// 3D PC will be convert to 2D.
// ATTENTION: The header file of this file is still tsm_node.hpp.
//            Because only use one file is too long for any readers.
//
// STEPS:
// 1. Use RANSAC to fit the circle to give ceres a good initial value.
// 2. Use ceres to optimize the circle center and radius.

#include "tsm/tsm_node.hpp"

namespace tsm
{

TsmNode::fitEachSegment(const PointCloudT::Ptr& seg, double center_x, Eigen::Vector3d& center, cv::Mat& debug_img)
{
  // 1. Project the 3D points to 2D plane (x-z plane) and get the 2D points.


  // 1. Use RANSAC to fit the circle to give ceres a good initial value.

  // 2. Use ceres to optimize the circle center and radius.
}
} // namespace tsm
