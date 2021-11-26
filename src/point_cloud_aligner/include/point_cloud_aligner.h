
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/package.h>

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

  std::vector<Eigen::Affine3d> vector_trans_optimized;


  std::string topic_name_target;
  std::string topic_name_source;
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> msg_target_;
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> msg_source_;
  std::shared_ptr<msg_target_> sub_lid_target_;
  std::shared_ptr<msg_source_> sub_lid_source_;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2> ApproxTime;
  typedef message_filters::Synchronizer<ApproxTime> msg_synchronizer_;
  std::shared_ptr<msg_synchronizer_> synchronizer_;
  void CallbackSaveData(const sensor_msgs::PointCloud2ConstPtr& msg_target,
                        const sensor_msgs::PointCloud2ConstPtr& msg_source);

  int mode;
  std::string path_pcd_folder;

  std::string path_target_cloud;
  std::string path_source_cloud;
  std::string path_output_file;
  std::vector<double> vector_q_param;
  std::vector<double> vector_trans_param;

};


