/*
 * Copyright (c) 2018 Intelligence Ally Technology Co., Ltd. All rights reserved.
 *
 * Created by Shaun Song on 8/9/18.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <glog/logging.h>
#include <ceres/gradient_checker.h>
#include <ceres/local_parameterization.h>
#include <ceres/sized_cost_function.h>

#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
using namespace std;
using namespace ceres;

template<typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
  ans << typename Derived::Scalar(0), -q(2), q(1),
      q(2), typename Derived::Scalar(0), -q(0),
      -q(1), q(0), typename Derived::Scalar(0);
  return ans;
}

template<typename QDerived, typename MDerived>
static Eigen::Matrix<typename QDerived::Scalar, 3, 4> DqvqDq(const Eigen::QuaternionBase<QDerived> &q,
                                                             const Eigen::MatrixBase<MDerived> &v) {
  Eigen::Matrix<typename QDerived::Scalar, 3, 4> ans;
  ans.template block<3, 3>(0, 0) = 2 * (
      -v * q.vec().transpose() + (v.dot(q.vec())) * Eigen::Matrix<typename QDerived::Scalar, 3, 3>::Identity()
          + q.vec() * v.transpose() - q.w() * skewSymmetric(v));
  ans.col(3) = 2 * (q.w() * v + q.vec().cross(v));
  return ans;
}

template<typename QDerived>
static Eigen::Matrix<typename QDerived::Scalar, 3, 3> DqvqDv(const Eigen::QuaternionBase<QDerived> &q) {
  return q.matrix();
}

template<typename QDerived>
Eigen::Matrix<typename QDerived::Scalar, 4, 4> Dq1q2Dq1(const Eigen::QuaternionBase<QDerived> &q2) {
  Eigen::Matrix<typename QDerived::Scalar, 4, 4, Eigen::RowMajor> jacobian;
  //Dq1q2Dq1
  jacobian.row(0) << q2.w(), q2.z(), -q2.y(), q2.x();
  jacobian.row(1) << -q2.z(), q2.w(), q2.x(), q2.y();
  jacobian.row(2) << q2.y(), -q2.x(), q2.w(), q2.z();
  jacobian.row(3) << -q2.x(), -q2.y(), -q2.z(), q2.w();
  return jacobian;
}

template<typename QDerived>
Eigen::Matrix<typename QDerived::Scalar, 4, 4> Dq1q2Dq2(const Eigen::QuaternionBase<QDerived> &q1) {
  Eigen::Matrix<typename QDerived::Scalar, 4, 4, Eigen::RowMajor> jacobian;
  //Dq1q2Dq2
  jacobian.row(0) << q1.w(), -q1.z(), q1.y(), q1.x();
  jacobian.row(1) << q1.z(), q1.w(), -q1.x(), q1.y();
  jacobian.row(2) << -q1.y(), q1.x(), q1.w(), q1.z();
  jacobian.row(3) << -q1.x(), -q1.y(), -q1.z(), q1.w();
  return jacobian;
}

template<typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta) {
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

class SophusSO3ApproximativeParameterization : public ceres::LocalParameterization {
  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Eigen::Quaterniond> _q(x);

//    Eigen::Quaterniond dq = Sophus::SO3d::exp(Eigen::Map<const Eigen::Vector3d>(delta)).unit_quaternion();
    Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta));
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);

    q = (_q * dq).normalized();

    return true;
  }
  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    j.block<3, 3>(0, 0).setIdentity();
    return true;
  }
  virtual int GlobalSize() const { return 4; };
  virtual int LocalSize() const { return 3; };
};

class SO3Transform : public ceres::SizedCostFunction<3, 4, 3, 3> {

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {

    const Eigen::Map<const Eigen::Quaterniond> quat(parameters[0]);
    const Eigen::Map<const Eigen::Vector3d> t(parameters[1]);
    const Eigen::Map<const Eigen::Vector3d> p(parameters[2]);

    Eigen::Map<Eigen::Vector3d> Tp(residuals);
    Tp = quat * p + t;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[0], 3, 4);

        jacobian.block<3, 3>(0, 0) = -quat.matrix() * skewSymmetric(p);
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[1], 3, 3);
        jacobian.setIdentity();
      }

      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[2], 3, 3);
        jacobian = DqvqDv(quat);
      }
    }

    return true;
  }
};

void SO3TransTest() {
  Sophus::SE3d pose = Sophus::SE3d::exp((Sophus::Vector6d() << 1.121, 2.5, 4.11, 1.4, .49, .41).finished());
  Eigen::Vector3d point{10, 14, 5};

  std::vector<const ceres::LocalParameterization *>
      local_params{new SophusSO3ApproximativeParameterization, nullptr, nullptr};
  NumericDiffOptions options;

  auto *cost = new SO3Transform();
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
}

int main(int argc, char **argv) {

  google::InitGoogleLogging(argv[0]);
  SO3TransTest();
  return 0;
}