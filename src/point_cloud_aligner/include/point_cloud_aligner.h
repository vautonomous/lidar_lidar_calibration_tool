
#include <ros/ros.h>
#include <point_to_plane_solver.h>
#include <point_cloud_handler.h>
#include <transform_handler.h>
#include <std_msgs/Int32.h>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include "point_cloud_aligner/Trans6Config.h"

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/impl/project_inliers.hpp>

#include <iostream>
#include <chrono>
#include <thread>


class PointCloudAligner {
 public:

  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  TransformHandler transform_handler_;

  ros::NodeHandle &nh_;
  explicit PointCloudAligner(ros::NodeHandle &nh);

  ros::Subscriber sub_key_;
  ros::Subscriber sub_clicked_point_;
  ros::Publisher pub_cloud_source_;
  ros::Publisher pub_cloud_aligned_;
  ros::Publisher pub_cloud_target_;
  ros::Publisher pub_cloud_stitched_;
  ros::Publisher pub_cloud_plane_source_;
  ros::Publisher pub_cloud_plane_target_;

  bool test;

  // MODE 0 - Visual Alignment:
  void CallbackVisualAlignment(point_cloud_aligner::Trans6Config &config, uint32_t level);
  dynamic_reconfigure::Server<point_cloud_aligner::Trans6Config> srv;
  dynamic_reconfigure::Server<point_cloud_aligner::Trans6Config>::CallbackType f;

  void CallbackPublishCloud(const ros::TimerEvent&);
  void KeyCallback(const std_msgs::Int32::ConstPtr& msg);
  void CallbackClickedPoint(const sensor_msgs::PointCloud2ConstPtr& msg_cloud_selected_points);

  void
  PublishCloud(const ros::Publisher &publisher,
                       const Cloud::Ptr &cloud,
                       const std::string &frame_id);

  Cloud::Ptr cloud_target;
  Cloud::Ptr cloud_source;
  Cloud::Ptr cloud_aligned;
  Cloud::Ptr cloud_stitched;

  Cloud::Ptr cloud_plane_points_source;
  Cloud::Ptr cloud_plane_points_target;
  std::vector<Eigen::Vector4d> vector_normal_coeff_source;
  std::vector<Eigen::Vector4d> vector_normal_coeff_target;

  std::vector<Eigen::Vector4f> vector_plane_coeff_target;
  std::vector<Cloud::Ptr> vector_plane_points_source;

  size_t pair_id;
  std::vector<std::string> path_pcd_target_list;
  std::vector<std::string> path_pcd_source_list;
  ros::Timer timer_pc_publisher_;

  bool is_optimization_done;
  std::vector<Eigen::Affine3d> vector_trans_optimized;

  // Test:
  std::vector<std::string> vector_path_test_pcd;
  std::vector<double> param_right_to_middle;
  std::vector<double> param_left_to_middle;

  ros::Publisher pub_cloud_middle;
  ros::Publisher pub_cloud_right_transformed;
  ros::Publisher pub_cloud_left_transformed;



};


