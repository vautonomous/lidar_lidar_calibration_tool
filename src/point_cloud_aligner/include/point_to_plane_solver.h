#ifndef SRC_POINT_CLOUD_ALIGNER_INCLUDE_POINT_TO_PLANE_SOLVER_H_
#define SRC_POINT_CLOUD_ALIGNER_INCLUDE_POINT_TO_PLANE_SOLVER_H_

#include <Eigen/Dense>
#include <iostream>

#include "openGA.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <transform_handler.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_plane.h>


class PointToPlaneSolver {
 public:
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;

  static Eigen::Matrix3d EulerToRotationMatrix(const double &yaw_degree,
                                               const double &pitch_degree,
                                               const double &roll_degree);
  static Eigen::Quaterniond EulerToQuaternion(const double &yaw_degree,
                                              const double &pitch_degree,
                                              const double &roll_degree);

  explicit PointToPlaneSolver(const std::vector<Eigen::Vector4f> &vector_plane_coeff_target_,
                              const std::vector<Cloud::Ptr> &vector_plane_points_source_);

  struct MySolution {
    double dx;
    double dy;
    double dz;
    double qx;
    double qy;
    double qz;
    double qw;

    std::string to_string() const {
      return
          std::string("{")
              + "dx:" + std::to_string(dx)
              + ", dy:" + std::to_string(dy)
              + ", dz:" + std::to_string(dz)
              + ", qx:" + std::to_string(qx)
              + ", qy:" + std::to_string(qy)
              + ", qz:" + std::to_string(qz)
              + ", qw:" + std::to_string(qw)
              + "}";
    }
  };

  struct MyMiddleCost {
    // This is where the results of simulation
    // is stored but not yet finalized.
    double objective1;
  };

  typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
  typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

  static void init_genes(MySolution &p, const std::function<double(void)> &rnd01);
  static bool eval_solution(const MySolution &p, MyMiddleCost &c);

  static MySolution mutate(const MySolution &X_base,
                           const std::function<double(void)> &rnd01,
                           double shrink_scale);

  static MySolution crossover(const PointToPlaneSolver::MySolution &X1,
                              const PointToPlaneSolver::MySolution &X2,
                              const std::function<double(void)> &rnd01);
  static double calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X);
  static void SO_report_generation(
      int generation_number,
      const EA::GenerationType<MySolution, MyMiddleCost> &last_generation,
      const MySolution &best_genes);

  Eigen::Affine3d solve();

};

#endif  // SRC_POINT_CLOUD_ALIGNER_INCLUDE_POINT_TO_PLANE_SOLVER_H_
