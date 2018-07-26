//
// Created by sxs on 7/26/18.
//

#include <glog/logging.h>
#include <ceres/gradient_checker.h>
#include <ceres/local_parameterization.h>
#include <ceres/sized_cost_function.h>

#include <sophus/se3.hpp>
using namespace std;
using namespace ceres;

Eigen::Matrix3d skewSymmetric(Eigen::Vector3d vec) {
  Eigen::Matrix3d mat;
  mat << 0, -vec(2), vec(1),
      vec(2), 0, -vec(0),
      -vec(1), vec(0), 0;
  return mat;
};
class QuatTransform : public ceres::SizedCostFunction<1, 4, 3, 3> {

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {

    const Eigen::Map<const Eigen::Quaterniond> quat(parameters[0]);
    const Eigen::Map<const Eigen::Vector3d> t(parameters[1]);
    const Eigen::Map<const Eigen::Vector3d> p(parameters[2]);

    Eigen::Vector3d tp = quat * p + t;
    residuals[0] = tp.squaredNorm();

    if (jacobians) {

      Eigen::Matrix<double, 1, 3> DrDtp = 2 * tp.transpose();
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[0], 1, 4);

        Eigen::Matrix<double, 3, 4> DqvqDq;
        DqvqDq.col(3) = 2 * (quat.w() * p + quat.vec().cross(p));

        DqvqDq.block<3, 3>(0, 0) = 2 * (-p * quat.vec().transpose() + (p.dot(quat.vec())) * Eigen::Matrix3d::Identity()
            + quat.vec() * p.transpose() - quat.w() * skewSymmetric(p));

        jacobian = DrDtp * DqvqDq;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[1], 1, 3);
        jacobian = DrDtp;
      }

      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[2], 1, 3);
        jacobian = DrDtp * quat.toRotationMatrix();
      }
    }

    return true;
  }
};

int main(int argc, char **argv) {

  google::InitGoogleLogging(argv[0]);

  Sophus::SE3d pose = Sophus::SE3d::exp((Sophus::Vector6d() << 1.121,2.5, 4.11, 1.4, .49, .41).finished());
  Eigen::Vector3d point{1444, 14, 5};

  std::vector<const ceres::LocalParameterization *>
      local_params{new ceres::EigenQuaternionParameterization, nullptr, nullptr};
  NumericDiffOptions options;

  auto *cost = new QuatTransform();
  ceres::GradientChecker checker(cost, &local_params, options);

  double *params[3]{pose.so3().data(), pose.translation().data(), point.data()};
  GradientChecker::ProbeResults results;
  if (!checker.Probe(params, 1e-11, &results)) {
    LOG(ERROR) << results.error_log;
  } else {
    for (int i = 0; i < results.local_jacobians.size(); ++i) {
      LOG(INFO) << "JACOBIAN #" << i << "\n" << results.local_jacobians[i] << "\n" << results.local_numeric_jacobians[i]
                << "\n" << results.local_jacobians[i] - results.local_numeric_jacobians[i];
    }
  }
  return 0;
}