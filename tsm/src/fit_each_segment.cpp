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

#include <random>

#include "tsm/tsm_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "ceres/ceres.h"

namespace tsm
{

// Ceres residual for circle fitting
// Auto diff
struct CircleFittingResidual
{
  CircleFittingResidual(float y, float z) : y_(y), z_(z) {}

  template <typename T>
  bool
  operator()(const T* const center_yz, const T* const radius, T* residual) const
  {
    T dy = T(y_) - center_yz[0];
    T dz = T(z_) - center_yz[1];
    residual[0] = ceres::sqrt(dy * dy + dz * dz) - radius[0];
    return true;
  }

private:
  const double y_;
  const double z_;
};

void TsmNode::fitEachSegment(const PointCloudT::Ptr& seg,
                             float                   center_x,
                             Eigen::Vector3f&        center,
                             cv::Mat&                debug_img)
{
  // 1. Project the 3D points to 2D plane (y-z plane) and get the 2D points.
  std::vector<Eigen::Vector2f> points_2d;
  for (const auto& pt : seg->points)
    points_2d.emplace_back(pt.y, pt.z);

  // 2. Use RANSAC to fit the circle to give ceres a good initial value.
  // -- Def a lambda function to compute a circle from three points.
  auto computeCircleFrom3Points = [](const Eigen::Vector2f& p1,
                                     const Eigen::Vector2f& p2,
                                     const Eigen::Vector2f& p3,
                                     Eigen::Vector2f&       center,
                                     float&                 radius) -> bool {
    float D = 2 * (p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) +
                   p3.x() * (p1.y() - p2.y()));
    // Points are collinear
    if (std::abs(D) < 1e-5)
      return false;
    float Ux = ((p1.squaredNorm()) * (p2.y() - p3.y()) +
                (p2.squaredNorm()) * (p3.y() - p1.y()) +
                (p3.squaredNorm()) * (p1.y() - p2.y())) /
               D;
    float Uy = ((p1.squaredNorm()) * (p3.x() - p2.x()) +
                (p2.squaredNorm()) * (p1.x() - p3.x()) +
                (p3.squaredNorm()) * (p2.x() - p1.x())) /
               D;
    center = Eigen::Vector2f(Ux, Uy);
    radius = (center - p1).norm();
    return true;
  };

  // -- Do RANSAC
  int                             best_inliers = 0;
  Eigen::Vector2f                 best_center;
  float                           best_radius = 0.0;
  int                             num_points = points_2d.size();
  std::mt19937                    rng{std::random_device{}()};
  std::uniform_int_distribution<> dist(0, num_points - 1);
  std::vector<Eigen::Vector2f>    best_inlier_pts;
  for (int i = 0; i < params_.cutting_fittting_.ransac_max_iterations; ++i)
  {
    // Randomly select 3 distinct points
    int i0 = dist(rng);
    int i1, i2;
    do
    {
      i1 = dist(rng);
    } while (i1 == i0);
    do
    {
      i2 = dist(rng);
    } while (i2 == i0 || i2 == i1);
    Eigen::Vector2f p1(points_2d[i0]);
    Eigen::Vector2f p2(points_2d[i1]);
    Eigen::Vector2f p3(points_2d[i2]);

    // Compute circle from the 3 points
    Eigen::Vector2f center;
    float           radius;
    if (!computeCircleFrom3Points(p1, p2, p3, center, radius))
      continue;

    // Count inliers
    int                          inliers = 0;
    std::vector<Eigen::Vector2f> inlier_pts;
    for (const auto& pt : points_2d)
    {
      float dist = std::abs((pt - center).norm() - radius);
      if (dist < params_.cutting_fittting_.ransac_distance_threshold)
      {
        inlier_pts.push_back(pt);
        ++inliers;
      }
    }

    // Update best model
    if (inliers > best_inliers)
    {
      best_inlier_pts = inlier_pts;
      best_inliers = inliers;
      best_center = center;
      best_radius = radius;
    }
  }

  // 4. Do Least Square fitting to refine the circle center and radius.
  // -- Def the initial values and optimization object.
  double center_yz[] = {best_center.x(), best_center.y()};
  double radius[] = {best_radius};

  // -- Def the problem and add residuals.
  ceres::Problem problem_g;

  // If residual bigger than value, the point will be treated as outlier and its
  // influence will be reduced.
  ceres::LossFunction* problem_loss_g = new ceres::HuberLoss(0.1);

  for (const auto& pt : best_inlier_pts)
  {
    problem_g.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CircleFittingResidual, 1, 2, 1>(
            new CircleFittingResidual(pt.x(), pt.y())),
        problem_loss_g,
        center_yz,
        radius);
  }

  // -- Config options and solve the problem.
  ceres::Solver::Options options_g;
  options_g.linear_solver_type = ceres::DENSE_QR;
  options_g.minimizer_progress_to_stdout = false;
  options_g.max_num_iterations = 50;
  ceres::Solver::Summary summary_g;
  ceres::Solve(options_g, &problem_g, &summary_g);

  center.y() = center_yz[0];
  center.z() = center_yz[1];
  center.x() = center_x;
  auto res_radius = radius[0];

  // 4. Draw the results on debug image
  const int img_size = 250;
  const int margin = 5;

  float view_half = best_radius * 1.5f;
  float scale = (img_size - 2 * margin) / (2.0f * view_half);
  float cy = center_yz[0];
  float cz = center_yz[1];

  auto toImg = [&](float y, float z) -> cv::Point2f {
    float px = img_size / 2.0f + (y - cy) * scale;
    float py = img_size / 2.0f - (z - cz) * scale;
    return cv::Point2f(px, py);
  };

  debug_img = cv::Mat::zeros(img_size, img_size, CV_8UC3);
  for (const auto& pt : points_2d)
    cv::circle(debug_img, toImg(pt.x(), pt.y()), 2, cv::Scalar(0, 255, 0), -1);

  cv::circle(debug_img,
             cv::Point2f(img_size / 2.0f, img_size / 2.0f),
             res_radius * scale,
             cv::Scalar(0, 0, 255),
             2);
  cv::circle(debug_img,
             cv::Point2f(img_size / 2.0f, img_size / 2.0f),
             3,
             cv::Scalar(255, 0, 0),
             -1);
}
} // namespace tsm
