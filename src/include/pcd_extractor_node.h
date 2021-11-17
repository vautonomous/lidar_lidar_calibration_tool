
#ifndef BUILD_POINTCLOUDNODE_H
#define BUILD_POINTCLOUDNODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/smart_ptr.hpp>
#include <pcl/common/common.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>

class PCDExtractorNode : public rclcpp::Node {
 public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  explicit PCDExtractorNode();
  size_t pair_id;

  // Point cloud and image:
  typedef sensor_msgs::msg::PointCloud2 typePc;
  using SyncPolicyT = message_filters::sync_policies::ApproximateTime<typePc, typePc>;
  message_filters::Subscriber<typePc> m_sub_target_cloud;
  message_filters::Subscriber<typePc> m_sub_source_cloud;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicyT>> my_synchronizer;

  void
  Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg_target,
           const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg_source);

  std::string path_pcd_folder;
  std::string topic_name_target;
  std::string topic_name_source;
};

#endif //BUILD_POINTCLOUDNODE_H
