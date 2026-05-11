///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

#include "tsm/tsm_node.hpp"

#include <chrono>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/pca.h"

#include "tf2_eigen/tf2_eigen.hpp"

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
}

void TsmNode::onPointClouds(const PC2::ConstSharedPtr& pc1,
                            const PC2::ConstSharedPtr& pc2)
{
  RCLCPP_INFO(get_logger(), "Received synced PointClouds");

  // Search TF from world to tube
  auto                                 stamp = pc1->header.stamp;
  geometry_msgs::msg::TransformStamped world_to_tube;
  geometry_msgs::msg::TransformStamped wrold_to_rgbd1;
  geometry_msgs::msg::TransformStamped wrold_to_rgbd2;
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

  try
  {
    wrold_to_rgbd1 =
        tf_buffer_->lookupTransform(params_.frames_.world_frame,
                                    params_.frames_.rgbd_1_frame,
                                    stamp,
                                    rclcpp::Duration::from_seconds(0.1));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
  }

  try
  {
    wrold_to_rgbd2 =
        tf_buffer_->lookupTransform(params_.frames_.world_frame,
                                    params_.frames_.rgbd_2_frame,
                                    stamp,
                                    rclcpp::Duration::from_seconds(0.1));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
  }

  // 1. Merge the two point clouds.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr c1(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr c2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc1, *c1);
  pcl::fromROSMsg(*pc2, *c2);

  Eigen::Matrix4f transform1 =
      tf2::transformToEigen(wrold_to_rgbd1.transform).matrix().cast<float>();
  Eigen::Matrix4f transform2 =
      tf2::transformToEigen(wrold_to_rgbd2.transform).matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::transformPointCloud(*c1, *c1, transform1);
  pcl::transformPointCloud(*c2, *c2, transform2);

  *merged = *c1 + *c2;

  // 2. Cut the merged point cloud to a box area around the tube to remove
  // outliers and background points.

  Eigen::Vector4f min_pt(params_.valid_pc_area_.x_min,
                         params_.valid_pc_area_.y_min,
                         params_.valid_pc_area_.z_min,
                         1.0);

  Eigen::Vector4f max_pt(params_.valid_pc_area_.x_max,
                         params_.valid_pc_area_.y_max,
                         params_.valid_pc_area_.z_max,
                         1.0);

  pcl::CropBox<pcl::PointXYZRGB> crop_box;
  crop_box.setMin(min_pt);
  crop_box.setMax(max_pt);
  crop_box.setInputCloud(merged);
  crop_box.filter(*merged);

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
  vgf.setLeafSize(0.002f, 0.002f, 0.002f);
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
  for (const auto & pt : aligned->points)
  {
    if (pt.x < x_actual_min) x_actual_min = pt.x;
    if (pt.x > x_actual_max) x_actual_max = pt.x;
  }
  double seg_len = params_.cutting_fittting_.length_of_each_segment;
  double x1min = x_actual_min;
  double x1max = x1min + seg_len;
  double xc    = (x_actual_min + x_actual_max) / 2.0;
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
    RCLCPP_WARN(get_logger(), "Seg1: %zu, Seg2: %zu, Seg3: %zu", seg1->size(), seg2->size(), seg3->size());
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

  // 7.
}
} // namespace tsm

RCLCPP_COMPONENTS_REGISTER_NODE(tsm::TsmNode)
