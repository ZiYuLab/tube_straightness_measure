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

namespace tsm
{

void TsmNode::postProcess()
{
  if (diff_bins_.size() < 3 || abs_bins_.size() < 3)
  {
    RCLCPP_WARN(
        get_logger(),
        "Not enough data for post-processing! So no data will be saved.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Post-processing...");

  std::vector<Eigen::Vector3f> center_points_wls_;
  std::vector<Eigen::Vector3f> center_points_integrator_;
  std::vector<Eigen::Vector3f> center_points_abs_;

  wls(diff_bins_, abs_bins_, center_points_wls_);
  integrator(diff_bins_, abs_bins_, center_points_integrator_);
  for (const auto& [idx, bd] : abs_bins_)
  {
    float x =
        (idx + 0.5f) * static_cast<float>(params_.integral_process_.bin_length);
    float y = bd.sum.y() / bd.count;
    float z = bd.sum.z() / bd.count;
    center_points_abs_.push_back(Eigen::Vector3f(x, y, z));
  }

  // By AI

  struct StraightnessResult
  {
    float                        straightness = 0.0f;
    std::vector<Eigen::Vector2f> plot_xy;
    std::vector<float>           distances;
  };

  auto evalStragainstOwnRefLine =
      [&](const std::vector<Eigen::Vector3f>& centers) -> StraightnessResult {
    StraightnessResult out;
    if (centers.size() < 2)
      return out;

    const Eigen::Vector3f ref_p0 = centers.front();
    const Eigen::Vector3f ref_p1 = centers.back();
    const Eigen::Vector3f ref_ex = ref_p1 - ref_p0;
    if (ref_ex.norm() < 1e-6f)
      return out;

    const Eigen::Vector3f ref_dir = ref_ex.normalized();
    Eigen::Vector3f       guide = Eigen::Vector3f::UnitY();
    if (std::abs(guide.dot(ref_dir)) > 0.95f)
      guide = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f ref_ey =
        (guide - guide.dot(ref_dir) * ref_dir).normalized();

    float y_max = std::numeric_limits<float>::lowest();

    out.plot_xy.reserve(centers.size());
    out.distances.reserve(centers.size());

    for (const auto& p : centers)
    {
      const Eigen::Vector3f rel = p - ref_p0;
      const float           x = rel.dot(ref_dir);

      // perp 是纯 3D 垂直偏差向量
      const Eigen::Vector3f perp = rel - x * ref_dir;

      // 1. 严格计算 3D 空间中的真实偏离距离
      const float dist = perp.norm();

      // 2. 根据投影方向判断符号 (与局部 Y 轴同向为正，反向为负)
      const float sign = (perp.dot(ref_ey) >= 0.0f) ? 1.0f : -1.0f;

      // 3. 组合成 带符号的真实距离
      const float signed_dist = dist * sign;

      out.plot_xy.emplace_back(x, signed_dist);
      out.distances.push_back(dist);

      y_max = std::max(y_max, dist);
    }

    // 最终的直线度评价值
    out.straightness = std::abs(y_max);
    return out;
  };

  auto saveCsv = [&](const std::string&        path,
                     const StraightnessResult& result) {
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
      RCLCPP_WARN(get_logger(), "Failed to open %s", path.c_str());
      return;
    }

    ofs << "x,signed_y,distance\n";
    for (size_t i = 0; i < result.plot_xy.size(); ++i)
      ofs << result.plot_xy[i].x() << "," << result.plot_xy[i].y() << ","
          << result.distances[i] << "\n";
  };

  const auto abs_result = evalStragainstOwnRefLine(center_points_abs_);
  const auto integrator_result =
      evalStragainstOwnRefLine(center_points_integrator_);
  const auto wls_result = evalStragainstOwnRefLine(center_points_wls_);

  RCLCPP_INFO(
      get_logger(), "Abs straightness: %.6f m", abs_result.straightness);
  RCLCPP_INFO(get_logger(),
              "Integrator straightness: %.6f m",
              integrator_result.straightness);
  RCLCPP_INFO(
      get_logger(), "WLS straightness: %.6f m", wls_result.straightness);

  const auto package_share_directory =
      ament_index_cpp::get_package_share_directory("tsm");
  const std::string& save_dir = package_share_directory + "/results";
  std::filesystem::create_directories(save_dir);

  saveCsv(save_dir + "/abs_track.csv", abs_result);
  saveCsv(save_dir + "/integrator_track.csv", integrator_result);
  saveCsv(save_dir + "/wls_track.csv", wls_result);
}

} // namespace tsm
