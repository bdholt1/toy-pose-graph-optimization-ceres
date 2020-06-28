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
  RelativeMotionError(double dx, double dy, double dtheta, Eigen::Matrix3d sqrt_information)
      : dx_(dx), dy_(dy), dtheta_(NormaliseAngle(dtheta)), sqrt_information_(sqrt_information) {}

  // calculate the error for each edge. a and b are 3-vectors representing state
  // of the node ie. x,y,theta
  template <typename T>
  bool operator()(const T* const a, const T* const b, T* e) const {
    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(e);

    // Convert a to isometric 2D transform of a in the world coordinate frame
    Eigen::Matrix<T, 3, 3> T_w_a = IsometricTransform2D<T>(a[0], a[1], a[2]);

    // Convert b to isometric 2D transform in the world coordinate frame
    Eigen::Matrix<T, 3, 3> T_w_b = IsometricTransform2D<T>(b[0], b[1], b[2]);

    // Convert observed position and orientation into isometric 2D transform
    Eigen::Matrix<T, 3, 3> T_a_b_hat = IsometricTransform2D<T>(T(dx_), T(dy_), T(dtheta_));

    // T_w_a^{-1} * T_w_b = T_a_w * T_w_b = T_a_b
    Eigen::Matrix<T, 3, 3> diff = T_a_b_hat.inverse() * (T_w_a.inverse() * T_w_b);

    residuals_map(0) = diff(0, 2);
    residuals_map(1) = diff(1, 2);
    residuals_map(2) = ceres::asin(NormaliseAngle(diff(1, 0)));

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta,
                                     const Eigen::Matrix3d& sqrt_information) {
    return (new ceres::AutoDiffCostFunction<RelativeMotionError, 3, 3, 3>(
        new RelativeMotionError(dx, dy, dtheta, sqrt_information)));
  };

 private:
  // the observed displacement of frame b w.r.t. frame a in the x direction
  const double dx_;
  // the observed displacement of frame b w.r.t. frame a in the y direction
  const double dy_;
  // the observed orientation of frame b w.r.t. frame a
  const double dtheta_;
  // the sqrt of the information matrix (inverse of covariance)
  const Eigen::Matrix3d sqrt_information_;
};

struct DCSLoopClosureError {
  // Observation for the edge
  DCSLoopClosureError(double dx, double dy, double dtheta, Eigen::Matrix3d sqrt_information)
      : dx_(dx), dy_(dy), dtheta_(NormaliseAngle(dtheta)), sqrt_information_(sqrt_information) {}

  // calculate the error for each edge. a and b are 3-vectors representing state
  // of the node ie. x,y,theta
  template <typename T>
  bool operator()(const T* const a, const T* const b, T* e) const {
    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(e);

    // Convert a to isometric 2D transform in the world coordinate frame
    Eigen::Matrix<T, 3, 3> T_w_a = IsometricTransform2D<T>(a[0], a[1], a[2]);

    // Convert b to isometric 2D transform in the world coordinate frame
    Eigen::Matrix<T, 3, 3> T_w_b = IsometricTransform2D<T>(b[0], b[1], b[2]);

    // Convert observed position and orientation into isometric 2D transform
    Eigen::Matrix<T, 3, 3> T_a_b_hat = IsometricTransform2D<T>(T(dx_), T(dy_), T(dtheta_));

    // T_w_a^{-1} * T_w_b = T_a_w * T_w_b = T_a_b
    Eigen::Matrix<T, 3, 3> diff = T_a_b_hat.inverse() * (T_w_a.inverse() * T_w_b);

    T res = diff(0, 2) * diff(0, 2) + diff(1, 2) * diff(1, 2);
    T psi_org = ceres::sqrt(T(2.0) * T(.5) / (T(.5) + res));
    T psi = std::min(T(1.0), psi_org);

    residuals_map(0) = psi * diff(0, 2);
    residuals_map(1) = psi * diff(1, 2);
    residuals_map(2) = psi * ceres::asin(NormaliseAngle(diff(1, 0)));

    return true;
  }

  static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta,
                                     const Eigen::Matrix3d& sqrt_information) {
    return (new ceres::AutoDiffCostFunction<DCSLoopClosureError, 3, 3, 3>(
        new DCSLoopClosureError(dx, dy, dtheta, sqrt_information)));
  };

 private:
  // the observed displacement of frame b w.r.t. frame a in the x direction
  const double dx_;
  // the observed displacement of frame b w.r.t. frame a in the y direction
  const double dy_;
  // the observed orientation of frame b w.r.t. frame a
  const double dtheta_;
  // the sqrt of the information matrix (inverse of covariance)
  const Eigen::Matrix3d sqrt_information_;
};

#endif  //_POSE_GRAPH_2D_ERROR_TERMS_H_
