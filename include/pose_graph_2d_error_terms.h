#ifndef _POSE_GRAPH_2D_ERROR_TERMS_H_
#define _POSE_GRAPH_2D_ERROR_TERMS_H_

#include <ceres/ceres.h>
#include <Eigen/Eigen>

#include "normalise_angle.h"

template <typename T>
Eigen::Matrix<T, 3, 3> IsometricTransform2D(T dx, T dy, T yaw_radians) {
  const T cos_yaw = ceres::cos(yaw_radians);
  const T sin_yaw = ceres::sin(yaw_radians);

  Eigen::Matrix<T, 3, 3> isometry2d;
  isometry2d << cos_yaw, -sin_yaw, dx, sin_yaw, cos_yaw, dy, T(0.0), T(0.0), T(1.0);
  return isometry2d;
}

struct RelativeMotionError {
  RelativeMotionError(const Eigen::Vector2d& p, const double theta, const Eigen::Matrix3d& sqrt_information)
      : measured_p_(p), measured_theta_(NormaliseAngle(theta)), sqrt_information_(sqrt_information) {}

  // calculate the error for each edge. a and b are 3-vectors representing state
  // of the node ie. x,y,theta
  template <typename T>
  bool operator()(const T* const p_a_ptr, const T* const theta_a_ptr, const T* const p_b_ptr,
                  const T* const theta_b_ptr, T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 2, 1> > p_a(p_a_ptr);
    const T theta_a = theta_a_ptr[0];

    Eigen::Map<const Eigen::Matrix<T, 2, 1> > p_b(p_b_ptr);
    const T theta_b = theta_b_ptr[0];

    // Convert a to isometric 2D transform of a in the world coordinate frame
    Eigen::Matrix<T, 3, 3> T_w_a = IsometricTransform2D<T>(p_a.x(), p_a.y(), theta_a);

    // Convert b to isometric 2D transform in the world coordinate frame
    Eigen::Matrix<T, 3, 3> T_w_b = IsometricTransform2D<T>(p_b.x(), p_b.y(), theta_b);

    // Convert observed position and orientation into isometric 2D transform
    Eigen::Matrix<T, 3, 3> T_a_b_hat =
        IsometricTransform2D<T>(T(measured_p_.x()), T(measured_p_.y()), T(measured_theta_));

    // T_w_a^{-1} * T_w_b = T_a_w * T_w_b = T_a_b
    Eigen::Matrix<T, 3, 3> diff = T_a_b_hat.inverse() * (T_w_a.inverse() * T_w_b);

    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(residuals_ptr);

    residuals(0) = diff(0, 2);
    residuals(1) = diff(1, 2);
    residuals(2) = ceres::asin(NormaliseAngle(diff(1, 0)));

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector2d p, const double theta,
                                     const Eigen::Matrix3d& sqrt_information) {
    return (new ceres::AutoDiffCostFunction<RelativeMotionError, 3, 2, 1, 2, 1>(
        new RelativeMotionError(p, theta, sqrt_information)));
  };

 private:
  // the observed displacement of frame b w.r.t. frame a
  const Eigen::Vector2d measured_p_;
  // the observed orientation of frame b w.r.t. frame a
  const double measured_theta_;
  // the sqrt of the information matrix (inverse of covariance)
  const Eigen::Matrix3d sqrt_information_;
};

struct DCSLoopClosureError {
  // Observation for the edge
  DCSLoopClosureError(const Eigen::Vector2d& p, const double theta, const Eigen::Matrix3d& sqrt_information)
      : measured_p_(p), measured_theta_(NormaliseAngle(theta)), sqrt_information_(sqrt_information) {}

  // calculate the error for each edge. a and b are 3-vectors representing state
  // of the node ie. x,y,theta
  template <typename T>
  bool operator()(const T* const p_a_ptr, const T* const theta_a_ptr, const T* const p_b_ptr,
                  const T* const theta_b_ptr, T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 2, 1> > p_a(p_a_ptr);
    const T theta_a = theta_a_ptr[0];

    Eigen::Map<const Eigen::Matrix<T, 2, 1> > p_b(p_b_ptr);
    const T theta_b = theta_b_ptr[0];

    // Convert a to isometric 2D transform of a in the world coordinate frame
    Eigen::Matrix<T, 3, 3> T_w_a = IsometricTransform2D<T>(p_a.x(), p_a.y(), theta_a);

    // Convert b to isometric 2D transform in the world coordinate frame
    Eigen::Matrix<T, 3, 3> T_w_b = IsometricTransform2D<T>(p_b.x(), p_b.y(), theta_b);

    // Convert observed position and orientation into isometric 2D transform
    Eigen::Matrix<T, 3, 3> T_a_b_hat =
        IsometricTransform2D<T>(T(measured_p_.x()), T(measured_p_.y()), T(measured_theta_));

    // T_w_a^{-1} * T_w_b = T_a_w * T_w_b = T_a_b
    Eigen::Matrix<T, 3, 3> diff = T_a_b_hat.inverse() * (T_w_a.inverse() * T_w_b);

    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(residuals_ptr);

    // see eq. 15 in "Robust Map Optimization using Dynamic Covariance Scaling", Agarwal et al 2013
    // residuals_map(0) = diff(0, 2);
    // residuals_map(1) = diff(1, 2);
    // residuals_map(2) = ceres::asin(NormaliseAngle(diff(1, 0)));

    // T chi_2  = residuals_map.transpose() * residuals_map;
    // T psi = T(0.5);
    // T s = std::min(T(1.0), T(2.0) * psi / (psi + chi_2));

    // Scale the residuals by the measurement uncertainty.
    // residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    // TODO: this is mpkuse's implementation. Find out why this works and the above does not.
    T res = diff(0, 2) * diff(0, 2) + diff(1, 2) * diff(1, 2);
    T psi_org = ceres::sqrt(T(2.0) * T(.5) / (T(.5) + res));
    T psi = std::min(T(1.0), psi_org);

    residuals(0) = psi * diff(0, 2);
    residuals(1) = psi * diff(1, 2);
    residuals(2) = psi * ceres::asin(NormaliseAngle(diff(1, 0)));

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector2d& p, const double theta,
                                     const Eigen::Matrix3d& sqrt_information) {
    return (new ceres::AutoDiffCostFunction<DCSLoopClosureError, 3, 2, 1, 2, 1>(
        new DCSLoopClosureError(p, theta, sqrt_information)));
  };

 private:
  // the observed displacement of frame b w.r.t. frame a
  const Eigen::Vector2d measured_p_;
  // the observed orientation of frame b w.r.t. frame a
  const double measured_theta_;
  // the sqrt of the information matrix (inverse of covariance)
  const Eigen::Matrix3d sqrt_information_;
};

#endif  //_POSE_GRAPH_2D_ERROR_TERMS_H_
