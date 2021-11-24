
#include "point_to_plane_solver.h"

double dx;
double dy;
double dz;
double qx;
double qy;
double qz;
double qw;

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

std::vector<Eigen::Vector4f> vector_plane_coeff_target;
std::vector<Cloud::Ptr> vector_plane_points_source;

PointToPlaneSolver::PointToPlaneSolver(
    const std::vector<Eigen::Vector4f> &vector_plane_coeff_target_,
    const std::vector<Cloud::Ptr> &vector_plane_points_source_)
{
  vector_plane_coeff_target = vector_plane_coeff_target_;
  vector_plane_points_source = vector_plane_points_source_;

  std::cout << "Total sample count: " << vector_plane_coeff_target.size() << std::endl;
}


Eigen::Matrix3d
PointToPlaneSolver::EulerToRotationMatrix(const double &yaw_degree,
                                   const double &pitch_degree,
                                   const double &roll_degree)
{
  Eigen::AngleAxisd rollAngle(roll_degree/57.29577, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch_degree/57.29577, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw_degree/57.29577, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q.toRotationMatrix();
}

Eigen::Quaterniond PointToPlaneSolver::EulerToQuaternion(const double &yaw_degree,
                                             const double &pitch_degree,
                                             const double &roll_degree)
{
  Eigen::AngleAxisd rollAngle(roll_degree/57.29577, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch_degree/57.29577, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw_degree/57.29577, Eigen::Vector3d::UnitZ());
  return yawAngle * pitchAngle * rollAngle;
}

struct MySolution
{
  double dx;
  double dy;
  double dz;
  double qx;
  double qy;
  double qz;
  double qw;

  std::string to_string() const
  {
    return
        std::string("{")
            +  "dx:"+std::to_string(dx)
            +", dy:"+std::to_string(dy)
            +", dz:"+std::to_string(dz)
            +", qx:"+std::to_string(qx)
            +", qy:"+std::to_string(qy)
            +", qz:"+std::to_string(qz)
            +", qw:"+std::to_string(qw)
            +"}";
  }
};

struct MyMiddleCost
{
  // This is where the results of simulation
  // is stored but not yet finalized.
  double objective1;
};

void
PointToPlaneSolver::init_genes(MySolution& p,const std::function<double(void)> &rnd01)
{
  // rnd01() gives a random number in 0~1
  p.dx=-12+24*rnd01();
  p.dy=-12+24*rnd01();
  p.dz=-12+24*rnd01();
  p.qx=-1+2*rnd01();
  p.qy=-1+2*rnd01();
  p.qz=-1+2*rnd01();
  p.qw=-1+2*rnd01();

}

bool
PointToPlaneSolver::eval_solution(
    const MySolution& p,
    MyMiddleCost &c)
{
  const double& dx_=p.dx;
  const double& dy_=p.dy;
  const double& dz_=p.dz;
  const double& qx_=p.qx;
  const double& qy_=p.qy;
  const double& qz_=p.qz;
  const double& qw_=p.qw;

  Eigen::Quaterniond q_predicted = {qw_, qx_, qy_, qz_};
  Eigen::Affine3d tf_predicted;
  tf_predicted.setIdentity();
  tf_predicted.linear() = q_predicted.toRotationMatrix();
  tf_predicted.translation() = Eigen::Vector3d{dx_, dy_, dz_};

  double cost = 0;
  std::vector<double> vector_costs;

  for (size_t i=0; i<vector_plane_coeff_target.size(); i++)
  {
    auto cloud_plane_points = vector_plane_points_source[i];
    auto plane_coeff_target = vector_plane_coeff_target[i];
    Cloud::Ptr cloud_plane_points_transformed(new Cloud);

    TransformHandler::transform_point_cloud(cloud_plane_points,
                                            cloud_plane_points_transformed,
                                            tf_predicted);

    double distance_total = 0;
    for(const auto& point : cloud_plane_points_transformed->points)
    {
      distance_total += pcl::pointToPlaneDistance(point, plane_coeff_target);
    }

    cost = cost + distance_total / cloud_plane_points_transformed->points.size();

    auto single_cost = distance_total /cloud_plane_points_transformed->points.size();
    vector_costs.push_back(single_cost);
  }

  double mean_cost = cost/vector_plane_coeff_target.size();

  c.objective1=mean_cost;
  return true; // solution is accepted
}

PointToPlaneSolver::MySolution
PointToPlaneSolver::mutate(
    const MySolution& X_base,
    const std::function<double(void)> &rnd01,
    double shrink_scale)
{
  MySolution X_new;
  const double mu = 0.2*shrink_scale; // mutation radius (adjustable)
  bool in_range;
  do{
    in_range=true;
    X_new=X_base;
    X_new.dx+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.dx>=-12 && X_new.dx<12);
    X_new.dy+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.dy>=-12 && X_new.dy<12);
    X_new.dz+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.dz>=-12 && X_new.dz<12);
    X_new.qx+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.qx>=-1 && X_new.qx<1);
    X_new.qy+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.qy>=-1 && X_new.qy<1);
    X_new.qz+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.qz>=-1 && X_new.qz<1);
    X_new.qw+=mu*(rnd01()-rnd01());
    in_range=in_range&&(X_new.qw>=-1 && X_new.qw<1);
  } while(!in_range);
  return X_new;
}

PointToPlaneSolver::MySolution PointToPlaneSolver::crossover(
    const MySolution& X1,
    const MySolution& X2,
    const std::function<double(void)> &rnd01)
{
  MySolution X_new;
  double r;
  r=rnd01();
  X_new.dx=r*X1.dx+(1.0-r)*X2.dx;
  r=rnd01();
  X_new.dy=r*X1.dy+(1.0-r)*X2.dy;
  r=rnd01();
  X_new.dz=r*X1.dz+(1.0-r)*X2.dz;
  r=rnd01();
  X_new.qx=r*X1.qx+(1.0-r)*X2.qx;
  r=rnd01();
  X_new.qy=r*X1.qy+(1.0-r)*X2.qy;
  r=rnd01();
  X_new.qz=r*X1.qz+(1.0-r)*X2.qz;
  r=rnd01();
  X_new.qw=r*X1.qw+(1.0-r)*X2.qw;
  return X_new;
}


double
PointToPlaneSolver::calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X)
{
  // finalize the cost
  double final_cost=0.0;
  final_cost+=X.middle_costs.objective1;
  return final_cost;
}


void PointToPlaneSolver::SO_report_generation(
    int generation_number,
    const EA::GenerationType<MySolution,MyMiddleCost> &last_generation,
    const MySolution& best_genes)
{
  std::cout
      <<"Generation ["<<generation_number<<"], "
      <<"Best="<<last_generation.best_total_cost<<", "
      <<"Average="<<last_generation.average_cost<<", "
      <<"Best genes=("<<best_genes.to_string()<<")"<<", "
      <<"Exe_time="<<last_generation.exe_time
      <<std::endl;

  dx = best_genes.dx;
  dy = best_genes.dy;
  dz = best_genes.dz;
  qx = best_genes.qx;
  qy = best_genes.qy;
  qz = best_genes.qz;
  qw = best_genes.qw;
}


Eigen::Affine3d
PointToPlaneSolver::solve()
{
  EA::Chronometer timer;
  timer.tic();

  GA_Type ga_obj;
  ga_obj.problem_mode=EA::GA_MODE::SOGA;
  ga_obj.multi_threading=true;
  ga_obj.verbose=false;
  ga_obj.population=500;
  ga_obj.generation_max=10000;
  ga_obj.calculate_SO_total_fitness=calculate_SO_total_fitness;
  ga_obj.init_genes=init_genes;
  ga_obj.eval_solution=eval_solution;
  ga_obj.mutate=mutate;
  ga_obj.crossover=crossover;
  ga_obj.SO_report_generation=SO_report_generation;
  ga_obj.crossover_fraction=0.7;
  ga_obj.mutation_rate=0.2;
  ga_obj.best_stall_max=10;
  ga_obj.elite_count=10;
  ga_obj.solve();

  Eigen::Quaterniond q_optimized = {qw, qx, qy, qz};
  Eigen::Affine3d tf_optimized;
  tf_optimized.setIdentity();
  tf_optimized.linear() = q_optimized.toRotationMatrix();
  tf_optimized.translation() = Eigen::Vector3d{dx, dy, dz};

  return tf_optimized;
}
