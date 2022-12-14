
#include "point_cloud_aligner.h"

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

PointCloudAligner::PointCloudAligner(ros::NodeHandle &nh)
    : nh_(nh), cloud_target(new Cloud), cloud_source(new Cloud),
      cloud_stitched(new Cloud),
      cloud_aligned(new Cloud), cloud_plane_points_source(new Cloud),
      cloud_plane_points_target(new Cloud),
      pair_id(0), mode(0) {

  nh_.getParam("path_pcd_target_list", path_pcd_target_list);
  nh_.getParam("path_pcd_source_list", path_pcd_source_list);
  nh_.getParam("topic_name_target", topic_name_target);
  nh_.getParam("topic_name_source", topic_name_source);
  nh_.getParam("mode", mode);
  nh_.getParam("path_pcd_folder", path_pcd_folder);

  nh_.getParam("path_target_cloud", path_target_cloud);
  nh_.getParam("path_source_cloud", path_source_cloud);
  nh_.getParam("path_output_file", path_output_file);
  nh_.getParam("q_optimized",vector_q_param);
  nh_.getParam("trans_optimized",vector_trans_param);

  if (mode == 0)
  {
    // Sample Saving Mode
    sub_lid_target_ = std::make_shared<msg_target_>(nh_, topic_name_target, 10);
    sub_lid_source_ = std::make_shared<msg_target_>(nh_, topic_name_source, 10);
    synchronizer_ = std::make_shared<msg_synchronizer_>(ApproxTime(10),
                                                        *sub_lid_target_,
                                                        *sub_lid_source_);
    synchronizer_->registerCallback(boost::bind(&PointCloudAligner::CallbackSaveData,
                                                this,
                                                _1, _2));
  } // eof Sample Saving Mode

  else if (mode == 1)
  {
    // Calibration Mode:
    sub_key_ = nh_.subscribe("/key", 30, &PointCloudAligner::KeyCallback, this);
    pub_cloud_source_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_source", 1, this);
    pub_cloud_aligned_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_aligned", 1, this);
    pub_cloud_target_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_target", 1, this);
    pub_cloud_stitched_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_stitched", 1, this);

    pub_cloud_plane_source_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_plane_source",
                                                                      1, this);
    pub_cloud_plane_target_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_plane_target",
                                                                      1, this);
    timer_pc_publisher_ = nh_.createTimer(ros::Duration(1),
                                          &PointCloudAligner::CallbackPublishCloud,
                                          this);
    sub_clicked_point_ = nh_.subscribe("/rviz_selected_points",
                                       30, &PointCloudAligner::CallbackClickedPoint,
                                       this);

    f = boost::bind(&PointCloudAligner::CallbackVisualAlignment, this, _1, _2);
    srv.setCallback(f);

  }  // eof Calibration Mode

  // PointCloud stitching mode
  else if (mode == 2)
  {
    std::cout << path_target_cloud << std::endl;
    std::cout << path_source_cloud << std::endl;
    std::cout << path_output_file << std::endl;
    pcl::io::loadPCDFile<Point>(path_target_cloud, *cloud_target);
    pcl::io::loadPCDFile<Point>(path_source_cloud, *cloud_source);
    Eigen::Affine3d tf;
    tf.setIdentity();
    Eigen::Quaterniond q = {vector_q_param[3], vector_q_param[0], vector_q_param[1], vector_q_param[2]};
    std::cout << "qx: " << q.x() << " qy: " << q.y() << " qz: " << q.z() << " qw: "<< q.w() << std::endl;
    tf.linear() = q.toRotationMatrix();
    tf.translation() = Eigen::Vector3d {vector_trans_param[0], vector_trans_param[1], vector_trans_param[2]};
    std::cout << "dx: " << tf.translation().x()
    << "dy: " << tf.translation().y() << "  dz:" << tf.translation().z() << std::endl;
    pcl::transformPointCloud(*cloud_source, *cloud_source, tf);
    *cloud_stitched += *cloud_target + *cloud_source;
    pcl::io::savePCDFile(path_output_file,*cloud_stitched);
    std::cout << "Stitched point cloud is saved." << std::endl;
  } // eof stitching mode

  Eigen::Affine3d trans_first_refinement;
  trans_first_refinement.setIdentity();
  vector_trans_optimized.push_back(trans_first_refinement);
}


void
PointCloudAligner::CallbackSaveData(const sensor_msgs::PointCloud2ConstPtr &msg_target,
                                             const sensor_msgs::PointCloud2ConstPtr &msg_source) {
  std::cout << "Message id: " << pair_id << std::endl;

  // Show time difference:
  std::cout << "Time diff: " << (msg_target->header.stamp - msg_source->header.stamp).toSec() << "s" << std::endl;
  // Make cloud message pcl type
  Cloud::Ptr cloud_target(new Cloud);
  pcl::fromROSMsg(*msg_target, *cloud_target);
  Cloud::Ptr cloud_source(new Cloud);
  pcl::fromROSMsg(*msg_source, *cloud_source);

  std::string pcd_path_target = path_pcd_folder + "/" + std::to_string(pair_id) + "_target.pcd";
  cloud_target->width = cloud_target->size();
  cloud_target->height = 1;
  cloud_target->is_dense = false;
  cloud_target->header.frame_id = "target_frame";
  pcl::io::savePCDFile(pcd_path_target, *cloud_target, false);

  std::string pcd_path_source = path_pcd_folder + "/" + std::to_string(pair_id) + "_source.pcd";
  cloud_source->width = cloud_source->size();
  cloud_source->height = 1;
  cloud_source->is_dense = false;
  cloud_source->header.frame_id = "target_frame";
  pcl::io::savePCDFile(pcd_path_source, *cloud_source, false);

  pair_id++;
}

void
PointCloudAligner::CallbackVisualAlignment(
    point_cloud_aligner::Trans6Config
    &config,
    uint32_t level) {

  transform_handler_.initial_transform_params.set_params(config.dx,
                                                         config.dy,
                                                         config.dz,
                                                         config.yaw,
                                                         config.pitch,
                                                         config.roll);

}

void
PointCloudAligner::KeyCallback(
    const std_msgs::Int32::ConstPtr &msg) {
  // press 'n' : next pair
  if (msg->data == 110) {
    std::cout << "\nNext pair." << std::endl;
    pair_id++;
    std::cout << path_pcd_target_list[pair_id] << std::endl;
    std::cout << path_pcd_source_list[pair_id] << std::endl;
  }

  // press 'p' : previous
  if (msg->data == 112 and pair_id != 0) {
    std::cout << "\nPrevious pair." << std::endl;
    pair_id--;
    std::cout << path_pcd_target_list[pair_id] << std::endl;
    std::cout << path_pcd_source_list[pair_id] << std::endl;
  }

  // press 'e' : to end
  if (msg->data == 101)
    ros::shutdown();

  // press 'd' : to discard last sample
  if (msg->data == 100) {
    if (!vector_plane_coeff_target.empty() and !vector_plane_points_source.empty()) {
      vector_plane_coeff_target.pop_back();
      vector_plane_points_source.pop_back();
      cloud_plane_points_target->points.clear();
      cloud_plane_points_source->points.clear();
      std::cout << "The last sample is discarded." << std::endl;
      std::cout << "Selected plane-points pair count: " <<
                vector_plane_coeff_target.size() << std::endl;
    } else {
      std::cout << "There is no is sample to be discarded." << std::endl;
    }

  }

  // press 'o' : to start optimization
  if (msg->data == 111) {
    PointToPlaneSolver point_to_plane_solver(vector_plane_coeff_target,
                                             vector_plane_points_source);
    Eigen::Affine3d trans_optimized = point_to_plane_solver.solve();
    vector_trans_optimized.push_back(trans_optimized);
    std::cout << "Optimization is done." << std::endl;

    Eigen::Affine3d trans_refinement_all = transform_handler_.initial_transform_params.get_tf();
    for (const auto &trans_optimized_curr : vector_trans_optimized)
      trans_refinement_all = trans_optimized_curr * trans_refinement_all;

    Eigen::Matrix3d rot_total = trans_refinement_all.rotation();
    Eigen::Quaterniond q_total(rot_total);
    q_total = q_total.normalized();

    std::cout << trans_refinement_all.translation().x() << " " <<
              trans_refinement_all.translation().y() << " " << trans_refinement_all.translation().z() <<
              " " << q_total.x() << " " << q_total.y() << " " << q_total.z() << " " <<
              q_total.w() << std::endl;
              
    Eigen::Vector3d ea = rot_total.eulerAngles(0, 1, 2); 
    std::cout << "roll: " << ea(0)*57.29577 << " pitch: " << ea(1)*57.29577 << " yaw: " << ea(2)*57.29577 << std::endl;

    vector_plane_coeff_target.clear();
    vector_plane_points_source.clear();
    cloud_plane_points_target->points.clear();
    cloud_plane_points_source->points.clear();
  }
}

void
PointCloudAligner::CallbackPublishCloud(const ros::TimerEvent &) {

  pcl::io::loadPCDFile<Point>(path_pcd_target_list[pair_id], *cloud_target);
  pcl::io::loadPCDFile<Point>(path_pcd_source_list[pair_id], *cloud_source);

  Cloud::Ptr cloud_tmp(new Cloud);
  TransformHandler::transform_point_cloud(cloud_source,
                                          cloud_tmp,
                                          transform_handler_.initial_transform_params.get_tf());

  Eigen::Affine3d trans_refinement_all;
  trans_refinement_all.setIdentity();
  for (const auto &trans_optimized : vector_trans_optimized)
    trans_refinement_all = trans_optimized * trans_refinement_all;

  TransformHandler::transform_point_cloud(cloud_tmp,
                                          cloud_aligned,
                                          trans_refinement_all);

  cloud_source->points.clear();
  pcl::copyPointCloud(*cloud_aligned, *cloud_source);

  *cloud_stitched += *cloud_target + *cloud_aligned;

  PublishCloud(pub_cloud_target_, cloud_target, "target_frame");
  PublishCloud(pub_cloud_source_, cloud_source, "target_frame");
  PublishCloud(pub_cloud_aligned_, cloud_aligned, "target_frame");
  PublishCloud(pub_cloud_stitched_, cloud_stitched, "target_frame");

  cloud_aligned->points.clear();
  cloud_stitched->points.clear();
}

void
PointCloudAligner::PublishCloud(const ros::Publisher &publisher,
                                const Cloud::Ptr &cloud,
                                const std::string &frame_id) {
  cloud->width = cloud->points.size();
  cloud->height = 1;
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud, msg_cloud);
  msg_cloud.header.frame_id = frame_id;
  msg_cloud.header.stamp = ros::Time::now();
  publisher.publish(msg_cloud);
}

void
PointCloudAligner::CallbackClickedPoint(
    const sensor_msgs::PointCloud2ConstPtr &msg_cloud_selected_points) {

  //std::cout << "Points are received. " << std::endl;
  Cloud::Ptr cloud_selected_points(new Cloud);
  pcl::fromROSMsg(*msg_cloud_selected_points,
                  *cloud_selected_points);

  //std::cout << "Selected point count: " << cloud_selected_points->points.size() << std::endl;

  Cloud::Ptr cloud_selected_points_target(new Cloud);
  Cloud::Ptr cloud_selected_points_source(new Cloud);

  PointCloudHandler::KeepApartSelectedPoints(cloud_selected_points,
                                             cloud_target,
                                             cloud_source,
                                             cloud_selected_points_target,
                                             cloud_selected_points_source);
  // Target Stuff:
  Eigen::Vector4d normal_coeff_target;
  Eigen::Vector4f plane_coeff_target;
  Point middle_point_target;
  Cloud::Ptr cloud_plane_target = PointCloudHandler::Points2Plane(cloud_selected_points_target,
                                                                  normal_coeff_target,
                                                                  plane_coeff_target,
                                                                  middle_point_target);
  *cloud_plane_points_target += *cloud_plane_target;
  vector_normal_coeff_target.push_back(normal_coeff_target);
  vector_plane_coeff_target.push_back(plane_coeff_target);

  PublishCloud(pub_cloud_plane_target_, cloud_plane_points_target, "target_frame");
  //std::cout << "Target plane point count: " << cloud_plane_target->points.size() << std::endl;

  // Source Stuff:
  Eigen::Vector4d normal_coeff_source;
  Eigen::Vector4f plane_coeff_source;
  Point middle_point_source;
  Cloud::Ptr cloud_plane_source = PointCloudHandler::Points2Plane(cloud_selected_points_source,
                                                                  normal_coeff_source,
                                                                  plane_coeff_source,
                                                                  middle_point_source);
  *cloud_plane_points_source += *cloud_plane_source;
  vector_normal_coeff_source.push_back(normal_coeff_source);
  vector_plane_points_source.push_back(cloud_plane_source);

  PublishCloud(pub_cloud_plane_source_, cloud_plane_points_source, "target_frame");

/*  std::cout << cloud_plane_points_source->points.size() << std::endl;
  std::cout << cloud_plane_points_target->points.size() << std::endl;*/

  double cost = 0;

  for (size_t i = 0; i < vector_plane_coeff_target.size(); i++) {
    auto cloud_plane_points = vector_plane_points_source[i];
    auto plane_coeff_target = vector_plane_coeff_target[i];

    double dist_average = 0;
    for (const auto &point : cloud_plane_points->points) {
      dist_average += pcl::pointToPlaneDistance(point, plane_coeff_target);
    }

    cost = cost + dist_average / cloud_plane_points->points.size();
    //std::cout << "Cost: " << dist_average / cloud_plane_points->points.size() << std::endl;
  }
  std::cout << "Selected plane-points pair count: " <<
            vector_plane_coeff_target.size() << std::endl;
}

