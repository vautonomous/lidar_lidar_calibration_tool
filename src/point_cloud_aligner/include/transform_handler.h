#ifndef SRC_POINT_CLOUD_ALIGNER_INCLUDE_TRANSFORM_HANDLER_H_
#define SRC_POINT_CLOUD_ALIGNER_INCLUDE_TRANSFORM_HANDLER_H_

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class TransformHandler {
 public:

  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  struct TransformParams {
    double dx = 0;
    double dy = 0;
    double dz = 0;
    double yaw = 0;
    double pitch = 0;
    double roll = 0;

    Eigen::Affine3d tf;


    void set_params(const double &dx_,
                    const double &dy_,
                    const double &dz_,
                    const double &yaw_,
                    const double &pitch_,
                    const double &roll_) {
      dx = dx_;
      dy = dy_;
      dz = dz_;
      yaw = yaw_;
      pitch = pitch_;
      roll = roll_;
    }

    Eigen::Quaterniond get_quaternion() {
      Eigen::AngleAxisd rollAngle(roll / 57.29577, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch / 57.29577, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw / 57.29577, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
      return q;
    }

    Eigen::Affine3d get_tf() {
      tf.linear() = get_quaternion().toRotationMatrix();
      tf.translation() = Eigen::Vector3d{dx, dy, dz};
      return tf;
    }

  };
  TransformParams initial_transform_params;
  TransformParams optimized_transform_params;

  static void transform_point_cloud(const Cloud::Ptr cloud_in,
                             Cloud::Ptr &cloud_out,
                             const Eigen::Affine3d& tf);

};

#endif  // SRC_POINT_CLOUD_ALIGNER_INCLUDE_TRANSFORM_HANDLER_H_
