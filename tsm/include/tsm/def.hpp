///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Just some shared definitions for the whole package.

#pragma once

#include "Eigen/Dense"

namespace tsm
{

struct Circle_t
{
  Eigen::Vector2d center_2d;
  Eigen::Vector3d center;
  double radius;
}

} // namespace tsm
