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

void TsmNode::integrator(const std::map<int, BinData>& diff_bins,
                         const std::map<int, BinData>& abs_bins,
                         std::vector<Eigen::Vector3f>& center_points)
{
  if (diff_bins.size() < 2)
    return;

  float bin_len = static_cast<float>(params_.integral_process_.bin_length);
  int   n = diff_bins.size();

  std::vector<float> x(n), ky(n), kz(n);
  int                i = 0;
  for (const auto& [idx, bd] : diff_bins)
  {
    x[i] = (idx + 0.5f) * bin_len;
    ky[i] = bd.sum.y() / bd.count;
    kz[i] = bd.sum.z() / bd.count;
    i++;
  }

  // First integration: curvature → slope (θ)
  std::vector<float> ty(n, 0.0f), tz(n, 0.0f);
  for (int j = 1; j < n; j++)
  {
    float dx = x[j] - x[j - 1];
    ty[j] = ty[j - 1] + (ky[j - 1] + ky[j]) * 0.5f * dx;
    tz[j] = tz[j - 1] + (kz[j - 1] + kz[j]) * 0.5f * dx;
  }

  // Second integration: slope → deflection (δ)
  std::vector<float> dy(n, 0.0f), dz(n, 0.0f);
  for (int j = 1; j < n; j++)
  {
    float dx = x[j] - x[j - 1];
    dy[j] = dy[j - 1] + (ty[j - 1] + ty[j]) * 0.5f * dx;
    dz[j] = dz[j - 1] + (tz[j - 1] + tz[j]) * 0.5f * dx;
  }

  center_points.resize(n);
  for (int j = 0; j < n; j++)
    center_points[j] = Eigen::Vector3f(x[j], dy[j], dz[j]);
}

} // namespace tsm
