#include "transform_handler.h"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

void TransformHandler::transform_point_cloud(const Cloud::Ptr cloud_in,
                           Cloud::Ptr &cloud_out,
                           const Eigen::Affine3d &tf) {
  for (const auto& point_in:cloud_in->points)
  {
    Eigen::Vector3d vec_point_out = tf * Eigen::Vector3d{point_in.x, point_in.y, point_in.z};
    Point point_out;
    point_out.x = static_cast<float>(vec_point_out(0));
    point_out.y = static_cast<float>(vec_point_out(1));
    point_out.z = static_cast<float>(vec_point_out(2));

    cloud_out->points.push_back(point_out);
  }
}