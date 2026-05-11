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
    std::vector<Eigen::Vector3f>&                         center_points)
{
  if (diff_bins.empty())
    return;

  // 1. Extract mean from each bin (map is already sorted by key = x order)
  float bin_len = static_cast<float>(params_.integral_process_.bin_length);
  std::vector<Eigen::Vector3f> binned;
  binned.reserve(diff_bins.size());
  for (const auto& [idx, sc] : diff_bins)
  {
    Eigen::Vector3f mean = sc.first / sc.second;
    mean.x()             = (idx + 0.5f) * bin_len;
    binned.push_back(mean);
  }

  // Apply filtfilt to y and z channels separately
  int                n = binned.size();

  // 2. Zero-phase moving average filter
  auto filtfilt = [&](const std::vector<float>& x) -> std::vector<float> {
    int half = std::max(1, n / 10);
    std::vector<float> fwd(n), bwd(n);
    for (int i = 0; i < n; i++)
    {
      int lo = std::max(0, i - half), hi = std::min(n - 1, i + half);
      float s = 0;
      for (int j = lo; j <= hi; j++) s += x[j];
      fwd[i] = s / (hi - lo + 1);
    }
    for (int i = n - 1; i >= 0; i--)
    {
      int lo = std::max(0, i - half), hi = std::min(n - 1, i + half);
      float s = 0;
      for (int j = lo; j <= hi; j++) s += fwd[j];
      bwd[i] = s / (hi - lo + 1);
    }
    return bwd;
  };

  // Apply filtfilt to y and z channels separately
  std::vector<float> ky(n), kz(n);
  for (int i = 0; i < n; i++)
  {
    ky[i] = binned[i].y();
    kz[i] = binned[i].z();
  }
  auto ky_f = filtfilt(ky);
  auto kz_f = filtfilt(kz);

  // Remove DC bias before integration to prevent quadratic drift
  auto removeDC = [&](std::vector<float>& v) {
    float mean = 0;
    for (float x : v) mean += x;
    mean /= v.size();
    for (float& x : v) x -= mean;
  };
  removeDC(ky_f);
  removeDC(kz_f);

  for (int i = 0; i < n; i++)
  {
    binned[i].y() = ky_f[i];
    binned[i].z() = kz_f[i];
  }

  // 3. Integrate twice to get centerline (trapezoid rule)
  std::vector<float> theta_y(n, 0.0f), theta_z(n, 0.0f);
  std::vector<float> delta_y(n, 0.0f), delta_z(n, 0.0f);

  for (int i = 1; i < n; i++)
  {
    float dx  = binned[i].x() - binned[i - 1].x();
    theta_y[i] = theta_y[i - 1] + (ky_f[i - 1] + ky_f[i]) / 2.0f * dx;
    theta_z[i] = theta_z[i - 1] + (kz_f[i - 1] + kz_f[i]) / 2.0f * dx;
  }
  for (int i = 1; i < n; i++)
  {
    float dx  = binned[i].x() - binned[i - 1].x();
    delta_y[i] = delta_y[i - 1] + (theta_y[i - 1] + theta_y[i]) / 2.0f * dx;
    delta_z[i] = delta_z[i - 1] + (theta_z[i - 1] + theta_z[i]) / 2.0f * dx;
  }

  // 4. Remove polynomial drift (quadratic detrend: a*x^2 + b*x + c)
  auto detrend = [&](std::vector<float>& v) {
    // Build normal equations for least-squares fit of y = a*x^2 + b*x + c
    // using centered x to avoid numerical issues
    double x_mean = 0;
    for (int i = 0; i < n; i++) x_mean += binned[i].x();
    x_mean /= n;

    double s1=0, s2=0, s3=0, s4=0, r0=0, r1=0, r2=0;
    for (int i = 0; i < n; i++)
    {
      double t = binned[i].x() - x_mean;
      double t2 = t * t;
      s1 += t; s2 += t2; s3 += t2 * t; s4 += t2 * t2;
      r0 += v[i]; r1 += t * v[i]; r2 += t2 * v[i];
    }
    // Normal equations: [n s1 s2; s1 s2 s3; s2 s3 s4] * [c b a]^T = [r0 r1 r2]^T
    // Solve 3x3 system via Cramer's rule
    Eigen::Matrix3d A;
    A << n,  s1, s2,
         s1, s2, s3,
         s2, s3, s4;
    Eigen::Vector3d rhs(r0, r1, r2);
    Eigen::Vector3d coef = A.ldlt().solve(rhs);
    double c = coef[0], b = coef[1], a = coef[2];

    for (int i = 0; i < n; i++)
    {
      double t = binned[i].x() - x_mean;
      v[i] -= static_cast<float>(a * t * t + b * t + c);
    }
  };
  detrend(delta_y);
  detrend(delta_z);

  center_points.resize(n);
  for (int i = 0; i < n; i++)
    center_points[i] = Eigen::Vector3f(binned[i].x(), delta_y[i], delta_z[i]);
}

} // namespace tsm