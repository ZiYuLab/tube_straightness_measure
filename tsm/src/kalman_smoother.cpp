///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Kalman smoother fusing curvature (2nd-order diff) and absolute center
// position to estimate tube deflection.
//
// State: X = [delta, theta, kappa]  (deflection, slope, curvature)
// Transition: kinematic model over spatial step dx
// Observations:
//   z1 = kappa   (from 2nd-order diff, H1 = [0,0,1])
//   z2 = delta   (from abs center pos after detrend, H2 = [1,0,0])

#include "tsm/tsm_node.hpp"

namespace tsm
{

std::vector<float> TsmNode::kalmanSmooth(const std::vector<float>& kappa,
                                         const std::vector<float>& abs_pos,
                                         const std::vector<float>& x_coords)
{
  int n = kappa.size();
  if (n < 2)
    return std::vector<float>(n, 0.0f);

  // Noise parameters (tunable)
  const double q_kappa = 1e-4;
  const double r_kappa = 1e-2;
  const double r_abs = 1.0; // large: abs position is noisy after detrend

  // Detrend abs_pos to remove installation offset (linear fit)
  double x_mean = 0, y_mean = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < n; i++)
  {
    x_mean += x_coords[i];
    y_mean += abs_pos[i];
  }
  x_mean /= n;
  y_mean /= n;
  for (int i = 0; i < n; i++)
  {
    double dx = x_coords[i] - x_mean;
    sxx += dx * dx;
    sxy += dx * abs_pos[i];
  }
  double             slope = (sxx > 1e-10) ? sxy / sxx : 0.0;
  double             intercept = y_mean - slope * x_mean;
  std::vector<float> abs_detrended(n);
  for (int i = 0; i < n; i++)
    abs_detrended[i] =
        abs_pos[i] - static_cast<float>(slope * x_coords[i] + intercept);

  // State and covariance
  Eigen::Vector3d X = Eigen::Vector3d::Zero();
  Eigen::Matrix3d P = Eigen::Matrix3d::Identity() * 1.0;

  // Process noise
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(2, 2) = q_kappa;

  // Observation matrices
  Eigen::RowVector3d H1(0, 0, 1); // curvature
  Eigen::RowVector3d H2(1, 0, 0); // deflection

  std::vector<float> result(n);

  for (int i = 0; i < n; i++)
  {
    if (i > 0)
    {
      double          dx = x_coords[i] - x_coords[i - 1];
      Eigen::Matrix3d F;
      // clang-format off
      F << 1, dx, 0.5 * dx * dx,
           0,  1,            dx,
           0,  0,             1;
      // clang-format on
      X = F * X;
      P = F * P * F.transpose() + Q;
    }

    // Update with curvature observation
    double          S1 = (H1 * P * H1.transpose())(0) + r_kappa;
    Eigen::Vector3d K1 = P * H1.transpose() / S1;
    X += K1 * (kappa[i] - (H1 * X)(0));
    P = (Eigen::Matrix3d::Identity() - K1 * H1) * P;

    // Update with absolute position observation
    double          S2 = (H2 * P * H2.transpose())(0) + r_abs;
    Eigen::Vector3d K2 = P * H2.transpose() / S2;
    X += K2 * (abs_detrended[i] - (H2 * X)(0));
    P = (Eigen::Matrix3d::Identity() - K2 * H2) * P;

    result[i] = static_cast<float>(X(0));
  }

  return result;
}

} // namespace tsm
