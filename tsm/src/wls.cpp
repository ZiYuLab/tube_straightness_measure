///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

#include "tsm/tsm_node.hpp"
#include "Eigen/Dense"

namespace tsm
{

void TsmNode::wls(const std::map<int, BinData>& diff_bins,
                  const std::map<int, BinData>& abs_bins,
                  std::vector<Eigen::Vector3f>& center_points)
{
  if (diff_bins.size() < 3)
    return;

  int          n = diff_bins.size();
  double       dx = params_.integral_process_.bin_length;
  double       dx2 = dx * dx;
  const double w_kappa = params_.integral_process_.w_kappa;
  const double w_abs = params_.integral_process_.w_abs;
  const double lambda = params_.integral_process_.lambda;

  // 1. Extract means
  std::vector<double> x(n), ky(n), kz(n), ay(n), az(n);
  {
    int i = 0;
    for (const auto& [idx, bd] : diff_bins)
    {
      x[i] = (idx + 0.5) * dx;
      ky[i] = bd.sum.y() / bd.count;
      kz[i] = bd.sum.z() / bd.count;
      i++;
    }
  }
  {
    int i = 0;
    for (const auto& [idx, bd] : abs_bins)
    {
      ay[i] = bd.sum.y() / bd.count;
      az[i] = bd.sum.z() / bd.count;
      i++;
    }
  }

  // 2. Detrend absolute position
  auto detrend = [&](std::vector<double>& v) {
    double x_mean = 0, y_mean = 0;
    for (int i = 0; i < n; i++)
    {
      x_mean += x[i];
      y_mean += v[i];
    }
    x_mean /= n;
    y_mean /= n;
    double sxx = 0, sxy = 0;
    for (int i = 0; i < n; i++)
    {
      double d = x[i] - x_mean;
      sxx += d * d;
      sxy += d * v[i];
    }
    double a = (sxx > 1e-10) ? sxy / sxx : 0.0;
    double b = y_mean - a * x_mean;
    for (int i = 0; i < n; i++)
      v[i] -= a * x[i] + b;
  };
  detrend(ay);
  detrend(az);

  // 3. Build system with fixed weights
  Eigen::MatrixXd AtWA = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd rhs_y = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd rhs_z = Eigen::VectorXd::Zero(n);

  // Curvature term
  for (int i = 0; i < n - 2; i++)
  {
    int    cols[3] = {i, i + 1, i + 2};
    double coef[3] = {1.0 / dx2, -2.0 / dx2, 1.0 / dx2};
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++)
        AtWA(cols[a], cols[b]) += w_kappa * coef[a] * coef[b];
    for (int a = 0; a < 3; a++)
    {
      rhs_y(cols[a]) += w_kappa * coef[a] * ky[i];
      rhs_z(cols[a]) += w_kappa * coef[a] * kz[i];
    }
  }

  // Absolute position term (endpoints boosted)
  for (int i = 0; i < n; i++)
  {
    double w = (i == 0 || i == n - 1) ? w_abs * 10.0 : w_abs;
    AtWA(i, i) += w;
    rhs_y(i) += w * ay[i];
    rhs_z(i) += w * az[i];
  }

  // Smoothness regularization
  for (int i = 0; i < n - 2; i++)
  {
    int    cols[3] = {i, i + 1, i + 2};
    double coef[3] = {1.0 / dx2, -2.0 / dx2, 1.0 / dx2};
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++)
        AtWA(cols[a], cols[b]) += lambda * coef[a] * coef[b];
  }

  // 4. Solve
  Eigen::VectorXd delta_y = AtWA.ldlt().solve(rhs_y);
  Eigen::VectorXd delta_z = AtWA.ldlt().solve(rhs_z);

  center_points.resize(n);
  for (int i = 0; i < n; i++)
    center_points[i] = Eigen::Vector3f(static_cast<float>(x[i]),
                                       static_cast<float>(delta_y(i)),
                                       static_cast<float>(delta_z(i)));
}

} // namespace tsm
