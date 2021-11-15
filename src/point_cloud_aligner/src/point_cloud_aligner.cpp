
#include "point_cloud_aligner.h"

PointCloudAligner::PointCloudAligner(ros::NodeHandle &nh)
    : nh_(nh), cloud_target(new Cloud), cloud_source(new Cloud),
      cloud_stitched(new Cloud),
      cloud_aligned(new Cloud), cloud_plane_points_source(new Cloud),
      cloud_plane_points_target(new Cloud), is_optimization_done(false),
      pair_id(0) {

  nh_.getParam("test", test);
  nh_.getParam("path_pcd_target_list", path_pcd_target_list);
  nh_.getParam("path_pcd_source_list", path_pcd_source_list);
  nh_.getParam("initial_dx", transform_handler_.initial_transform_params.dx);
  nh_.getParam("initial_dy", transform_handler_.initial_transform_params.dy);
  nh_.getParam("initial_dz", transform_handler_.initial_transform_params.dz);
  nh_.getParam("initial_yaw", transform_handler_.initial_transform_params.yaw);
  nh_.getParam("initial_pitch", transform_handler_.initial_transform_params.pitch);
  nh_.getParam("initial_roll", transform_handler_.initial_transform_params.roll);

  nh_.getParam("param_right_to_middle", param_right_to_middle);
  nh_.getParam("param_left_to_middle", param_left_to_middle);
  nh_.getParam("path_test_pcd", vector_path_test_pcd);

  if (!test) {
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


    Eigen::Affine3d trans_first_refinement;
    trans_first_refinement.setIdentity();
    vector_trans_optimized.push_back(trans_first_refinement);

  } else {

    Eigen::Affine3d tf;
    tf.setIdentity();
    Eigen::Quaterniond q = {0.996732, -0.00242802, -0.000187559, -0.0807386};
    tf.linear() = q.toRotationMatrix();
    tf.translation() = Eigen::Vector3d {0.0867071, -1.19197, -2.27943};

    std::string id = "b3";
    std::string path_target = "/home/goktug/projects/lidar_lidar_calibration_tool_new/src/lidar_lidar_calibration_tool/pcd_files/otokar/01_and_05_stitched/" + id + ".pcd";
    std::string path_source = "/home/goktug/projects/lidar_lidar_calibration_tool_new/src/lidar_lidar_calibration_tool/pcd_files/otokar/02_to_05/" + id + "/0_source_cloud.pcd";

    Cloud::Ptr cloud_target_(new Cloud);
    Cloud::Ptr cloud_source_(new Cloud);
    Cloud::Ptr cloud_aligned_(new Cloud);
    Cloud::Ptr cloud_stitched_(new Cloud);

    pcl::io::loadPCDFile<Point>(path_target, *cloud_target_);
    pcl::io::loadPCDFile<Point>(path_source, *cloud_source_);
    TransformHandler::transform_point_cloud(cloud_source_, cloud_aligned_, tf);

    *cloud_stitched_ += *cloud_target_ + *cloud_aligned_;

    std::string file_name = "/home/goktug/Desktop/" + id + ".pcd";
    pcl::io::savePCDFile(file_name,*cloud_stitched_);


    /*std::cout << "Test is started." << std::endl;
    pub_cloud_middle = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_middle", 1, this);
    pub_cloud_right_transformed = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_right_transformed", 1, this);
    pub_cloud_left_transformed = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_left_transformed", 1, this);

    Cloud::Ptr cloud_middle(new Cloud);
    Cloud::Ptr cloud_right(new Cloud);
    Cloud::Ptr cloud_right_transformed(new Cloud);
    Cloud::Ptr cloud_left(new Cloud);
    Cloud::Ptr cloud_left_transformed(new Cloud);

    pcl::io::loadPCDFile<Point>(vector_path_test_pcd[0], *cloud_middle);
    pcl::io::loadPCDFile<Point>(vector_path_test_pcd[1], *cloud_right);
    pcl::io::loadPCDFile<Point>(vector_path_test_pcd[2], *cloud_left);

    Eigen::Affine3d trans_right_to_middle;
    trans_right_to_middle.setIdentity();
    Eigen::Quaterniond q_right_to_middle = {param_right_to_middle[6], param_right_to_middle[3],
                                            param_right_to_middle[4], param_right_to_middle[5]};
    trans_right_to_middle.linear() = q_right_to_middle.toRotationMatrix();
    trans_right_to_middle.translation() = Eigen::Vector3d{param_right_to_middle[0],
                                                          param_right_to_middle[1],
                                                          param_right_to_middle[2]};

    Eigen::Affine3d trans_left_to_middle;
    trans_left_to_middle.setIdentity();
    Eigen::Quaterniond q_left_to_middle = {param_left_to_middle[6], param_left_to_middle[3],
                                           param_left_to_middle[4], param_left_to_middle[5]};
    trans_left_to_middle.linear() = q_left_to_middle.toRotationMatrix();
    trans_left_to_middle.translation() = Eigen::Vector3d{param_left_to_middle[0],
                                                         param_left_to_middle[1],
                                                         param_left_to_middle[2]};

    TransformHandler::transform_point_cloud(cloud_right,
                                            cloud_right_transformed,
                                            trans_right_to_middle);

    TransformHandler::transform_point_cloud(cloud_left,
                                            cloud_left_transformed,
                                            trans_left_to_middle);

    for (size_t i = 0; i < 2000; i++) {
      PublishCloud(pub_cloud_middle, cloud_middle, "target_frame");
      PublishCloud(pub_cloud_left_transformed, cloud_left_transformed, "target_frame");
      PublishCloud(pub_cloud_right_transformed, cloud_right_transformed, "target_frame");
      std::chrono::milliseconds timespan(2000);
      std::this_thread::sleep_for(timespan);
    }*/




  }


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
  if (msg->data == 112 and pair_id!=0) {
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

