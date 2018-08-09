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

        jacobian = DrDtp * DqvqDq(quat, p);
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[1], 1, 3);
        jacobian = DrDtp;
      }

      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[2], 1, 3);
        jacobian = DrDtp * DqvqDv(quat);
      }
    }

    return true;
  }
};

void QuatTransTest() {
  Sophus::SE3d pose = Sophus::SE3d::exp((Sophus::Vector6d() << 1.121, 2.5, 4.11, 1.4, .49, .41).finished());
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
}

class QuatPlus : public ceres::SizedCostFunction<4, 4, 4> {

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {

    const Eigen::Map<const Eigen::Quaterniond> q1(parameters[0]);
    const Eigen::Map<const Eigen::Quaterniond> q2(parameters[1]);
    Eigen::Map<Eigen::Quaterniond> q1q2(residuals);

    q1q2 = q1 * q2;

    if (jacobians) {

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[0], 4, 4);
        //Dq1q2Dq1
        jacobian = Dq1q2Dq1(q2);
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[1], 4, 4);
        //Dq1q2Dq2
        jacobian = Dq1q2Dq2(q1);
      }
    }

    return true;
  }
};

void QuatPlusTest() {
  Sophus::SE3d pose = Sophus::SE3d::exp((Sophus::Vector6d() << 1.121, 2.5, 4.11, 1.4, .49, .41).finished());
  Eigen::Vector3d point{1444, 14, 5};

  Eigen::Quaterniond quat1 = Eigen::Quaterniond::UnitRandom();
  Eigen::Quaterniond quat2 = Eigen::Quaterniond::UnitRandom();

  std::vector<const ceres::LocalParameterization *>
      local_params{nullptr, nullptr};
  NumericDiffOptions options;

  auto *cost = new QuatPlus();
  ceres::GradientChecker checker(cost, &local_params, options);

  double *params[2]{quat1.coeffs().data(), quat2.coeffs().data()};
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

class QuatSlerp : public ceres::SizedCostFunction<4, 4, 4, 1> {

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const {

    const Eigen::Map<const Eigen::Quaterniond> q1(parameters[0]);
    const Eigen::Map<const Eigen::Quaterniond> q2(parameters[1]);
    const double t = *parameters[2];
    Eigen::Map<Eigen::Quaterniond> result(residuals);

    result = q1.slerp(t, q2);

    const double one = 1 - Eigen::NumTraits<double>::epsilon();
    double d = q1.dot(q2);
    double absD = Eigen::numext::abs(d);

    double scale0;
    double scale1;

    double theta, sinTheta, cosTheta;
    if (absD >= one) {
      scale0 = 1 - t;
      scale1 = t;
    } else {
      // theta is the angle between the 2 quaternions
      theta = Eigen::numext::acos(absD);
      sinTheta = Eigen::numext::sin(theta);
      cosTheta = Eigen::numext::cos(theta);

      scale0 = Eigen::numext::sin((1 - t) * theta) / sinTheta;
      scale1 = Eigen::numext::sin((t * theta)) / sinTheta;
    }
    if (d < 0) scale1 = -scale1;

    result.coeffs() = scale0 * q1.coeffs() + scale1 * q2.coeffs();

    if (jacobians) {
      double Dscale0Dtheta = ((1 - t) * sinTheta * Eigen::numext::cos((1 - t) * theta)
          - Eigen::numext::sin((1 - t) * theta) * cosTheta)
          / pow(sinTheta, 2);
      double Dscale1Dtheta = (t * Eigen::numext::cos(t * theta) * sinTheta - Eigen::numext::sin(t * theta) * cosTheta)
          / pow(sinTheta, 2);
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[0], 4, 4);

        Eigen::Matrix<double, 1, 4> Dscale0Dq1, Dscale1Dq1, DthetaDq1;

        if (absD >= one) {
          Dscale0Dq1.setZero();
          Dscale1Dq1.setZero();
        } else {
          DthetaDq1 = -1 / Eigen::numext::sqrt(1 - pow(d, 2)) * q2.coeffs().transpose();
          if (d < 0) DthetaDq1 *= -1;   // this for abs()
          Dscale0Dq1 = Dscale0Dtheta * DthetaDq1;
          Dscale1Dq1 = Dscale1Dtheta * DthetaDq1;

          if (d < 0) Dscale1Dq1 *= -1;
        }

        jacobian = q1.coeffs() * Dscale0Dq1 + q2.coeffs() * Dscale1Dq1 + Eigen::MatrixXd::Identity(4, 4) * scale0;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[1], 4, 4);
        //Dq1q2Dq2
        Eigen::Matrix<double, 1, 4> Dscale0Dq2, Dscale1Dq2, DthetaDq2;

        if (absD >= one) {
          Dscale0Dq2.setZero();
          Dscale1Dq2.setZero();
        } else {
          DthetaDq2 = -1 / Eigen::numext::sqrt(1 - pow(d, 2)) * q1.coeffs().transpose();
          if (d < 0) DthetaDq2 *= -1;   // this for abs()
          Dscale0Dq2 = Dscale0Dtheta * DthetaDq2;
          Dscale1Dq2 = Dscale1Dtheta * DthetaDq2;

          if (d < 0) Dscale1Dq2 *= -1;
        }

        jacobian = q1.coeffs() * Dscale0Dq2 + q2.coeffs() * Dscale1Dq2 + Eigen::MatrixXd::Identity(4, 4) * scale1;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[2], 1, 4);
        double Dscale0Dt, Dscale1Dt;
        if (absD >= one) {
          Dscale0Dt = -1;
          Dscale1Dt = 1;
        } else {
          Dscale0Dt = -theta * Eigen::numext::cos((1 - t) * theta) / sinTheta;
          Dscale1Dt = theta * Eigen::numext::cos(t * theta) / sinTheta;
        }
        if (d < 0) Dscale1Dt *= -1;

        jacobian = q1.coeffs().transpose() * Dscale0Dt + q2.coeffs().transpose() * Dscale1Dt;
      }
    }

    return true;
  }
};

void QuatSlerpTest() {
  Eigen::Quaterniond quat1 = Eigen::Quaterniond::UnitRandom();
  Eigen::Quaterniond quat2 = Eigen::Quaterniond::UnitRandom();
  double tao = 0.1234;
  std::vector<const ceres::LocalParameterization *>
      local_params{nullptr, nullptr, nullptr};
  NumericDiffOptions options;

  auto *cost = new QuatSlerp();
  ceres::GradientChecker checker(cost, &local_params, options);

  double *params[3]{quat1.coeffs().data(), quat2.coeffs().data(), &tao};
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
//  QuatTransTest();
//  QuatPlusTest();
  QuatSlerpTest();
  return 0;
}