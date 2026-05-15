///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Just integrate the second order difference.

#include "tsm/tsm_node.hpp"

namespace tsm
{

void TsmNode::integrator(const std::vector<BinProj>&   projected,
                         std::vector<Eigen::Vector2f>& center_points)
{
  int n = static_cast<int>(projected.size());
  if (n < 2)
    return;

  // First integration: kappa → slope
  std::vector<float> t(n, 0.0f);
  for (int i = 1; i < n; i++)
  {
    float du = projected[i].u - projected[i - 1].u;
    t[i] = t[i - 1] + (projected[i - 1].kappa + projected[i].kappa) * 0.5f * du;
  }

  // Second integration: slope → deflection
  center_points.resize(n);
  center_points[0] = Eigen::Vector2f(projected[0].u, 0.0f);
  for (int i = 1; i < n; i++)
  {
    float du = projected[i].u - projected[i - 1].u;
    float d  = center_points[i - 1].y() + (t[i - 1] + t[i]) * 0.5f * du;
    center_points[i] = Eigen::Vector2f(projected[i].u, d);
  }
}

} // namespace tsm
