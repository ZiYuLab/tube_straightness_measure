///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

// Post-process

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <filesystem>

#include "tsm/tsm_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "Eigen/Dense"

namespace tsm
{

void TsmNode::postProcess()
{
  if (bins_.size() < 3)
  {
    RCLCPP_WARN(
        get_logger(),
        "Not enough data for post-processing! So no data will be saved.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Post-processing...");

  // 1. Collect per-bin mean absolute position and kappa.
  int                          n = static_cast<int>(bins_.size());
  std::vector<Eigen::Vector3f> pts(n);
  std::vector<float>           w(n), kappas(n);
  {
    int i = 0;
    for (const auto& [idx, bd] : bins_)
    {
      pts[i] = bd.sum_c / static_cast<float>(bd.count);
      w[i] = static_cast<float>(bd.count);
      kappas[i] = static_cast<float>(bd.sum_kappa / bd.count);
      i++;
    }
  }

  // 2. Weighted centroid.
  float           W = 0.0f;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (int i = 0; i < n; i++)
  {
    centroid += w[i] * pts[i];
    W += w[i];
  }
  centroid /= W;

  // 3. Weighted covariance → PCA.
  Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
  for (int i = 0; i < n; i++)
  {
    Eigen::Vector3f q = pts[i] - centroid;
    C += w[i] * q * q.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(C / W);
  // Eigenvalues ascending: col(2)=axis direction, col(1)=primary bending.
  Eigen::Vector3f e1 = eig.eigenvectors().col(2);
  Eigen::Vector3f e2 = eig.eigenvectors().col(1);

  // 4. Project onto (e1, e2), shift first point to origin.
  std::vector<BinProj> projected(n);
  for (int i = 0; i < n; i++)
  {
    Eigen::Vector3f q = pts[i] - centroid;
    projected[i] = {e1.dot(q), e2.dot(q), kappas[i]};
  }
  const float u0 = projected[0].u, v0 = projected[0].v;
  for (auto& p : projected)
  {
    p.u -= u0;
    p.v -= v0;
  }

  // Evaluate straightness against the line connecting first and last point,
  // save CSV, and log the result.
  auto evalAndSave = [&](const std::vector<Eigen::Vector2f>& pts,
                         const std::string&                  path) {
    if (pts.size() < 2)
      return;

    const Eigen::Vector2f p0 = pts.front();
    const Eigen::Vector2f p1 = pts.back();
    const Eigen::Vector2f dir = (p1 - p0).normalized();
    const Eigen::Vector2f nor(-dir.y(), dir.x()); // left-hand normal

    std::ofstream ofs(path);
    ofs << "u,proj,dist\n";

    float max_dist = 0.0f;
    for (const auto& p : pts)
    {
      float proj = (p - p0).dot(dir);
      float dist = std::abs((p - p0).dot(nor)); // un signed
      ofs << p.x() << "," << proj << "," << dist << "\n";
      max_dist = std::max(max_dist, dist);
    }

    RCLCPP_INFO(
        get_logger(), "Straightness [%s]: %.6f m", path.c_str(), max_dist);
  };

  const auto        pkg = ament_index_cpp::get_package_share_directory("tsm");
  const std::string save_dir = pkg + "/results";
  std::filesystem::create_directories(save_dir);

  std::vector<Eigen::Vector2f> abs_pts(n), integrator_pts, wls_pts;
  for (int i = 0; i < n; i++)
    abs_pts[i] = Eigen::Vector2f(projected[i].u, projected[i].v);

  integrator(projected, integrator_pts);
  wls(projected, wls_pts);

  evalAndSave(abs_pts, save_dir + "/abs_track.csv");
  evalAndSave(integrator_pts, save_dir + "/integrator_track.csv");
  evalAndSave(wls_pts, save_dir + "/wls_track.csv");
}

} // namespace tsm
