///////////////////////////////////////////////////////////
//                                                       //
// Author: ziyedeyuu@163.com (Zhaoyu Chen)               //
// License: GPL License                                  //
//                                                       //
///////////////////////////////////////////////////////////

#pragma once

#include <memory>

#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/opencv.hpp"

namespace tsm
{

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;

class TsmNode : public rclcpp::Node
{
private:
  using PC2 = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PC2, PC2>;

public:
  explicit TsmNode(const rclcpp::NodeOptions& options);

private:
  void setupParams();
  void setupTf();
  void setupPub();
  void setupSub();
  void onPointClouds(const PC2::ConstSharedPtr& pc1,
                     const PC2::ConstSharedPtr& pc2);
  void fitEachSegment(const PointCloudT::Ptr& seg, double center_x,
    Eigen::Vector3d& center, cv::Mat& debug_img);

  struct
  {
    struct
    {
      std::string rgbd_1_pointcloud;
      std::string rgbd_2_pointcloud;
      std::string merged_pointcloud;
      std::string seg1_pointcloud;
      std::string seg2_pointcloud;
      std::string seg3_pointcloud;
    } topics_;

    struct
    {
      std::string rgbd_1_frame;
      std::string rgbd_2_frame;
      std::string tube_frame;
      std::string world_frame;
    } frames_;

    // Valid PC area
    // Cut the point cloud to a box area around the tube to remove outliers and
    // background points.
    struct
    {
      double x_min = -0.5;
      double x_max = 0.5;
      double y_min = -0.5;
      double y_max = 0.5;
      double z_min = -0.5;
      double z_max = 0.5;
    } valid_pc_area_;

    struct
    {
      // Minimum points
      int min_points_in_segment = 200;

      // Each segment will be how long along the tube axis.
      double length_of_each_segment = 0.1;
    } cutting_fittting_;

  } params_;

  message_filters::Subscriber<PC2>                           rgbd_1_sub_;
  message_filters::Subscriber<PC2>                           rgbd_2_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  std::shared_ptr<tf2_ros::Buffer>                           tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>                tf_listener_;
  rclcpp::Publisher<PC2>::SharedPtr                          merged_pc_pub_;
  rclcpp::Publisher<PC2>::SharedPtr                          seg1_pc_pub_;
  rclcpp::Publisher<PC2>::SharedPtr                          seg2_pc_pub_;
  rclcpp::Publisher<PC2>::SharedPtr                          seg3_pc_pub_;
};

} // namespace tsm
