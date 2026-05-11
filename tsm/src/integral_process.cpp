///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// This file is used to integral the second order difference data to get the
// tube centerline.
// ATTENTION: The header file of this file is still tsm_node.hpp.
//            Because only use one file is too long for any readers.
//
// STEPS:
//

#include <algorithm>
#include <cmath>
#include <map>

#include "tsm/tsm_node.hpp"

namespace tsm
{

void TsmNode::integralProcess(
    const std::map<int, std::pair<Eigen::Vector3f, int>>& diff_bins,
    const std::map<int, std::pair<Eigen::Vector3f, int>>& abs_bins,
    std::vector<Eigen::Vector3f>&                         center_points)
{
  if (diff_bins.empty())
    return;

  float bin_len = static_cast<float>(params_.integral_process_.bin_length);

  // Extract mean curvature and absolute position for bins present in diff_bins
  int                  n = diff_bins.size();
  std::vector<float>   x_coords(n), ky(n), kz(n), ay(n), az(n);
  int                  i = 0;
  for (const auto& [idx, sc] : diff_bins)
  {
    x_coords[i] = (idx + 0.5f) * bin_len;
    ky[i]       = sc.first.y() / sc.second;
    kz[i]       = sc.first.z() / sc.second;

    auto it = abs_bins.find(idx);
    if (it != abs_bins.end())
    {
      ay[i] = it->second.first.y() / it->second.second;
      az[i] = it->second.first.z() / it->second.second;
    }
    else
    {
      ay[i] = 0.0f;
      az[i] = 0.0f;
    }
    i++;
  }

  auto delta_y = kalmanSmooth(ky, ay, x_coords);
  auto delta_z = kalmanSmooth(kz, az, x_coords);

  center_points.resize(n);
  for (int j = 0; j < n; j++)
    center_points[j] = Eigen::Vector3f(x_coords[j], delta_y[j], delta_z[j]);
}

} // namespace tsm