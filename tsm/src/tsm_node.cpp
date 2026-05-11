///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

#include "tsm/tsm_node.hpp"

#include <chrono>
#include <future>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/pca.h"

#include "tf2_eigen/tf2_eigen.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace tsm
{

TsmNode::TsmNode(const rclcpp::NodeOptions& options) : Node("tsm_node", options)
{
  setupParams();
  setupTf();
  setupSub();
  setupPub();
}

void TsmNode::setupParams()
{
  params_.topics_.rgbd_1_pointcloud = declare_parameter<std::string>(
      "topics.rgbd_1_pointcloud", "rgbd_1/points");
  params_.topics_.rgbd_2_pointcloud = declare_parameter<std::string>(
      "topics.rgbd_2_pointcloud", "rgbd_2/points");
  params_.topics_.merged_pointcloud = declare_parameter<std::string>(
      "topics.merged_pointcloud", "merged_points");
  params_.topics_.seg1_pointcloud =
      declare_parameter<std::string>("topics.seg1_pointcloud", "seg1_points");
  params_.topics_.seg2_pointcloud =
      declare_parameter<std::string>("topics.seg2_pointcloud", "seg2_points");
  params_.topics_.seg3_pointcloud =
      declare_parameter<std::string>("topics.seg3_pointcloud", "seg3_points");
  params_.topics_.debug_image =
      declare_parameter<std::string>("topics.debug_image", "debug_image");
  params_.topics_.centerline_marker =
      declare_parameter<std::string>("topics.centerline_marker", "centerline");

  params_.frames_.rgbd_1_frame = declare_parameter<std::string>(
      "frames.rgbd_1_frame", "rgbd_1/camera_link/rgbd_1");
  params_.frames_.rgbd_2_frame = declare_parameter<std::string>(
      "frames.rgbd_2_frame", "rgbd_2/camera_link/rgbd_2");
  params_.frames_.tube_frame =
      declare_parameter<std::string>("frames.tube_frame", "tube");
  params_.frames_.world_frame =
      declare_parameter<std::string>("frames.world_frame", "world");

  params_.valid_pc_area_.x_min =
      declare_parameter<double>("valid_pc_area.x_min", -0.5);
  params_.valid_pc_area_.x_max =
      declare_parameter<double>("valid_pc_area.x_max", 0.5);
  params_.valid_pc_area_.y_min =
      declare_parameter<double>("valid_pc_area.y_min", -0.5);
  params_.valid_pc_area_.y_max =
      declare_parameter<double>("valid_pc_area.y_max", 0.5);
  params_.valid_pc_area_.z_min =
      declare_parameter<double>("valid_pc_area.z_min", -0.5);
  params_.valid_pc_area_.z_max =
      declare_parameter<double>("valid_pc_area.z_max", 0.5);

  params_.cutting_fittting_.length_of_each_segment =
      declare_parameter<double>("cutting_fittting.length_of_each_segment", 0.1);
  params_.cutting_fittting_.ransac_distance_threshold =
      declare_parameter<double>("cutting_fittting.ransac_distance_threshold",
                                0.005);
  params_.cutting_fittting_.ransac_max_iterations =
      declare_parameter<int>("cutting_fittting.ransac_max_iterations", 100);
  params_.cutting_fittting_.min_points_in_segment =
      declare_parameter<int>("cutting_fittting.min_points_in_segment", 200);

  params_.integral_process_.bin_length = declare_parameter<double>(
      "integral_process.bin_length",
      params_.cutting_fittting_.length_of_each_segment);
}

void TsmNode::setupTf()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void TsmNode::setupSub()
{
  // Sub PC
  rgbd_1_sub_.subscribe(this, params_.topics_.rgbd_1_pointcloud);
  rgbd_2_sub_.subscribe(this, params_.topics_.rgbd_2_pointcloud);

  // Use ApproximateTime policy to synchronize the two point cloud topics
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), rgbd_1_sub_, rgbd_2_sub_);
  sync_->registerCallback(std::bind(&TsmNode::onPointClouds,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
}

void TsmNode::setupPub()
{
  merged_pc_pub_ = create_publisher<PC2>(params_.topics_.merged_pointcloud, 10);
  seg1_pc_pub_ = create_publisher<PC2>(params_.topics_.seg1_pointcloud, 10);
  seg2_pc_pub_ = create_publisher<PC2>(params_.topics_.seg2_pointcloud, 10);
  seg3_pc_pub_ = create_publisher<PC2>(params_.topics_.seg3_pointcloud, 10);
  debug_img_pub_ = create_publisher<sensor_msgs::msg::Image>(
      params_.topics_.debug_image, 10);
  centerline_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      params_.topics_.centerline_marker, 10);
}

void TsmNode::onPointClouds(const PC2::ConstSharedPtr& pc1,
                            const PC2::ConstSharedPtr& pc2)
{
  RCLCPP_INFO(get_logger(), "Received synced PointClouds");

  // Search TF from world to tube
  auto                                 stamp = pc1->header.stamp;
  geometry_msgs::msg::TransformStamped world_to_tube;
  try
  {
    world_to_tube =
        tf_buffer_->lookupTransform(params_.frames_.world_frame,
                                    params_.frames_.tube_frame,
                                    stamp,
                                    rclcpp::Duration::from_seconds(0.1));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
  }

  // 1. Merge the two point clouds — transform directly into merged.
  if (!static_tf_ready_)
  {
    try
    {
      transform_rgbd1_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
                                                   params_.frames_.world_frame,
                                                   params_.frames_.rgbd_1_frame,
                                                   tf2::TimePointZero))
                             .matrix()
                             .cast<float>();
      transform_rgbd2_ = tf2::transformToEigen(tf_buffer_->lookupTransform(
                                                   params_.frames_.world_frame,
                                                   params_.frames_.rgbd_2_frame,
                                                   tf2::TimePointZero))
                             .matrix()
                             .cast<float>();
      static_tf_ready_ = true;
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(get_logger(), "Static TF not ready: %s", ex.what());
      return;
    }
  }

  PointCloudT::Ptr merged(new PointCloudT);
  {
    PointCloudT::Ptr c1(new PointCloudT), c2(new PointCloudT);
    auto f = std::async(std::launch::async,
                        [&] { pcl::fromROSMsg(*pc2, *c2); });
    pcl::fromROSMsg(*pc1, *c1);
    f.wait();
    pcl::transformPointCloud(*c1, *c1, transform_rgbd1_);
    pcl::transformPointCloud(*c2, *c2, transform_rgbd2_);
    *merged = *c1 + *c2;
  }

  // 2. Cut the merged point cloud to a box area around the tube.
  {
    float xmin = params_.valid_pc_area_.x_min, xmax = params_.valid_pc_area_.x_max;
    float ymin = params_.valid_pc_area_.y_min, ymax = params_.valid_pc_area_.y_max;
    float zmin = params_.valid_pc_area_.z_min, zmax = params_.valid_pc_area_.z_max;
    merged->erase(
        std::remove_if(merged->begin(), merged->end(), [&](const PointT& p) {
          return p.x < xmin || p.x > xmax || p.y < ymin || p.y > ymax ||
                 p.z < zmin || p.z > zmax;
        }),
        merged->end());
  }

  // 3. Sent merged and filter result to ros2 (DEBUG)
  PC2 merged_msg;
  pcl::toROSMsg(*merged, merged_msg);
  merged_msg.header.stamp = pc1->header.stamp;
  merged_msg.header.frame_id = params_.frames_.world_frame;
  merged_pc_pub_->publish(merged_msg);

  // 4. Find the True tube axis direction using PCA.

  // -- Voxel Grid Filtering to make the dese of point cloud more uniform and
  //    reduce noise.
  PointCloudT::Ptr processed(new PointCloudT);

  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(merged);
  vgf.setLeafSize(0.015f, 0.015f, 0.015f);
  vgf.filter(*processed);

  // TODO: Minimum points
  // -- DO PCA
  pcl::PCA<PointT> pca;
  pca.setInputCloud(processed);
  auto eigen_vectors = pca.getEigenVectors();

  // -- If the length is smaller than the width, the first eigen value is not
  //    the tube axis direction. We should swap to second eigen vector.
  Eigen::Vector3f tube_axis_direction;

  float dot_product = eigen_vectors.col(0).dot(Eigen::Vector3f::UnitX());
  if (std::abs(dot_product) > 0.707) // col(0) aligns with X → tube axis
    tube_axis_direction = eigen_vectors.col(0);
  else
    tube_axis_direction = eigen_vectors.col(1);

  // -- Rotate the point cloud to make the tube axis direction align with x
  //    axis.
  Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(
      tube_axis_direction, Eigen::Vector3f::UnitX());

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();

  PointCloudT::Ptr aligned(new PointCloudT);
  pcl::transformPointCloud(*merged, *aligned, transform);

  // 5. Try to cut them to three segments along the x axis.
  PointCloudT::Ptr seg1(new PointCloudT);
  PointCloudT::Ptr seg2(new PointCloudT);
  PointCloudT::Ptr seg3(new PointCloudT);

  float x_actual_min = std::numeric_limits<float>::max();
  float x_actual_max = std::numeric_limits<float>::lowest();
  for (const auto& pt : aligned->points)
  {
    if (pt.x < x_actual_min)
      x_actual_min = pt.x;
    if (pt.x > x_actual_max)
      x_actual_max = pt.x;
  }
  double seg_len = params_.cutting_fittting_.length_of_each_segment;
  double x1min = x_actual_min;
  double x1max = x1min + seg_len;
  double xc = (x_actual_min + x_actual_max) / 2.0;
  double x2min = xc - seg_len / 2.0;
  double x2max = xc + seg_len / 2.0;
  double x3max = x_actual_max;
  double x3min = x3max - seg_len;

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(aligned);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x1min, x1max);
  pass.filter(*seg1);
  pass.setFilterLimits(x2min, x2max);
  pass.filter(*seg2);
  pass.setFilterLimits(x3min, x3max);
  pass.filter(*seg3);

  // -- If there are too few points in one segment, return.
  if (seg1->size() < 50 || seg2->size() < 50 || seg3->size() < 50)
  {
    RCLCPP_WARN(get_logger(),
                "Seg1: %zu, Seg2: %zu, Seg3: %zu",
                seg1->size(),
                seg2->size(),
                seg3->size());
    RCLCPP_WARN(get_logger(), "Too few points in one segment, skipping...");
    return;
  }

  // 6. Publish the three segments (DEBUG)
  auto publishSeg = [&](PointCloudT::Ptr&                  cloud,
                        rclcpp::Publisher<PC2>::SharedPtr& pub) {
    PC2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = pc1->header.stamp;
    msg.header.frame_id = params_.frames_.world_frame;
    pub->publish(msg);
  };
  publishSeg(seg1, seg1_pc_pub_);
  publishSeg(seg2, seg2_pc_pub_);
  publishSeg(seg3, seg3_pc_pub_);

  // 7. Fit each segment to find the circle center and radius (parallel).
  Eigen::Vector3f center1, center2, center3;
  cv::Mat         debug_img1, debug_img2, debug_img3;
  auto            f1 = std::async(std::launch::async, [&] {
    fitEachSegment(seg1, (x1min + x1max) / 2.0, center1, debug_img1);
  });
  auto            f2 = std::async(std::launch::async, [&] {
    fitEachSegment(seg2, (x2min + x2max) / 2.0, center2, debug_img2);
  });
  fitEachSegment(seg3, (x3min + x3max) / 2.0, center3, debug_img3);
  f1.wait();
  f2.wait();

  // 8. Compute second order difference for this frame.
  Eigen::Vector2f m1 = center1.segment<2>(1);
  Eigen::Vector2f m2 = center2.segment<2>(1);
  Eigen::Vector2f m3 = center3.segment<2>(1);
  float           d = center2.x() - center1.x();
  Eigen::Vector2f ms = (m3 - 2 * m2 + m1) / (d * d);

  Eigen::Vector3f ms_3d(center2.x(), ms.x(), ms.y());

  RCLCPP_INFO(get_logger(),
              "Frame curvature: (%.3f, %.3f), center2: (%.3f, %.3f, %.3f)",
              ms.x(),
              ms.y(),
              center2.x(),
              center2.y(),
              center2.z());

  // 9. transform the 2-diff data to tube frame.
  Eigen::Matrix3f R_pca = transform.block<3, 3>(0, 0);
  Eigen::Affine3f world_to_tube_f =
      tf2::transformToEigen(world_to_tube.transform).cast<float>();

  Eigen::Vector3f ms_world =
      R_pca.transpose() * Eigen::Vector3f(0, ms.x(), ms.y());
  Eigen::Vector3f ms_tube = world_to_tube_f.linear() * ms_world;

  float pos_tube_x = (world_to_tube_f * (R_pca.transpose() * center2)).x();

  // Absolute center position in tube frame (y, z only)
  Eigen::Vector3f center2_tube =
      world_to_tube_f * (R_pca.transpose() * center2);

  RCLCPP_INFO(get_logger(),
              "(%.3f, %.3f, %.3f)",
              center2_tube.x(),
              ms_tube.y(),
              ms_tube.z());

  // Reject outlier curvature values (threshold: 10 m^-2)
  if (std::abs(ms_tube.y()) > 10.0f || std::abs(ms_tube.z()) > 10.0f)
  {
    RCLCPP_WARN(get_logger(),
                "Outlier curvature rejected: (%.3f, %.3f)",
                ms_tube.y(),
                ms_tube.z());
  }
  else
  {
    int bin_idx = static_cast<int>(
        std::floor(pos_tube_x / params_.integral_process_.bin_length));
    {
      auto& [sum, count] = diff_bins_[bin_idx];
      sum += Eigen::Vector3f(pos_tube_x, ms_tube.y(), ms_tube.z());
      count++;
    }
    {
      auto& [sum, count] = abs_bins_[bin_idx];
      sum += Eigen::Vector3f(pos_tube_x, center2_tube.y(), center2_tube.z());
      count++;
    }
  }

  //   if (!diff_bins_.empty())
  //     integralProcess(diff_bins_, abs_bins_, center_points_);

  // 11. Draw circle fitting results for each segment (DEBUG)
  cv::putText(debug_img1,
              "seg1",
              cv::Point(5, 15),
              cv::FONT_HERSHEY_SIMPLEX,
              0.5,
              cv::Scalar(255, 255, 255),
              1);
  cv::putText(debug_img2,
              "seg2",
              cv::Point(5, 15),
              cv::FONT_HERSHEY_SIMPLEX,
              0.5,
              cv::Scalar(255, 255, 255),
              1);
  cv::putText(debug_img3,
              "seg3",
              cv::Point(5, 15),
              cv::FONT_HERSHEY_SIMPLEX,
              0.5,
              cv::Scalar(255, 255, 255),
              1);

  cv::Mat combined;
  cv::hconcat(std::vector<cv::Mat>{debug_img1, debug_img2, debug_img3},
              combined);

  // Publish debug image
  auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header{}, "bgr8", combined)
                     .toImageMsg();
  img_msg->header.stamp = pc1->header.stamp;
  debug_img_pub_->publish(*img_msg);

  // Publish centerline marker
  //   if (!center_points_.empty())
  //   {
  //     Eigen::Affine3f tube_to_world = world_to_tube_f.inverse();

  //     visualization_msgs::msg::Marker marker;
  //     marker.header.stamp = pc1->header.stamp;
  //     marker.header.frame_id = params_.frames_.world_frame;
  //     marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  //     marker.action = visualization_msgs::msg::Marker::ADD;
  //     marker.scale.x = 0.01;
  //     marker.color.r = 1.0f;
  //     marker.color.a = 1.0f;
  //     for (const auto& pt : center_points_)
  //     {
  //       Eigen::Vector3f           pw = tube_to_world * pt;
  //       geometry_msgs::msg::Point p;
  //       p.x = pw.x();
  //       p.y = pw.y();
  //       p.z = pw.z();
  //       marker.points.push_back(p);
  //     }
  //     RCLCPP_INFO(get_logger(),
  //                 "Marker points: %zu, first: (%.3f, %.3f, %.3f)",
  //                 marker.points.size(),
  //                 marker.points.empty() ? 0.0 : marker.points.front().x,
  //                 marker.points.empty() ? 0.0 : marker.points.front().y,
  //                 marker.points.empty() ? 0.0 : marker.points.front().z);
  //     centerline_pub_->publish(marker);
  //   }
}
} // namespace tsm

RCLCPP_COMPONENTS_REGISTER_NODE(tsm::TsmNode)
