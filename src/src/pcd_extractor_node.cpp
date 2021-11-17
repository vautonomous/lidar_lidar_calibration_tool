//
// Created by goktug on 6.04.2021.
//

#include "pcd_extractor_node.h"

using namespace std::chrono_literals;

PCDExtractorNode::PCDExtractorNode()
: Node("pcd_extractor"),
  pair_id(0)
{
  this->declare_parameter("path_pcd_folder");
  rclcpp::Parameter param_camera_names = this->get_parameter("path_pcd_folder");
  path_pcd_folder = param_camera_names.as_string();
  std::cout << path_pcd_folder << std::endl;

  this->declare_parameter("topic_name_target");
  rclcpp::Parameter param_topic_name_target = this->get_parameter("topic_name_target");
  topic_name_target = param_topic_name_target.as_string();
  std::cout << topic_name_target << std::endl;

  this->declare_parameter("topic_name_source");
  rclcpp::Parameter param_topic_name_source = this->get_parameter("topic_name_source");
  topic_name_source = param_topic_name_source.as_string();
  std::cout << topic_name_source << std::endl;

  m_sub_target_cloud.subscribe(this,"/lidar_front/velodyne_points",rclcpp::QoS(100).get_rmw_qos_profile());
  m_sub_source_cloud.subscribe(this,"/lidar_left/velodyne_points", rclcpp::QoS(100).get_rmw_qos_profile());
  my_synchronizer = std::make_unique<message_filters::Synchronizer<SyncPolicyT>>(
      SyncPolicyT(30), m_sub_target_cloud, m_sub_source_cloud);
  my_synchronizer->registerCallback(
      std::bind(
          &PCDExtractorNode::Callback,
          this, std::placeholders::_1,
          std::placeholders::_2));

}


void
PCDExtractorNode::Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_target,
                           const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_source)
{
  std::cout << "Message id: " << pair_id << std::endl;

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

