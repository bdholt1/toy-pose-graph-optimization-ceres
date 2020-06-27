#ifndef _POSE_GRAPH_2D_ERROR_TERMS_H_
#define _POSE_GRAPH_2D_ERROR_TERMS_H_

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
  RelativeMotionError(double dx, double dy, double dtheta)
      : dx_(dx), dy_(dy), dtheta_(NormaliseAngle(dtheta)) {}

  // calculate the error for each edge. a and b are 3-vectors representing state of the node ie. x,y,theta
  template <typename T>
  bool operator()(const T* const a, const T* const b, T* e) const {
    // Convert a to T1 ^w_T_a
    Eigen::Matrix<T, 3, 3> w_T_a = IsometricTransform2D<T>(a[0], a[1], a[2]);

    // Convert b to T2 ^w_T_a
    Eigen::Matrix<T, 3, 3> w_T_b = IsometricTransform2D<T>(b[0], b[1], b[2]);

    // Convert observed transform a_Tcap_b
    Eigen::Matrix<T, 3, 3> a_Tcap_b = IsometricTransform2D<T>(T(dx_), T(dy_), T(dtheta_));

    // now we have :: w_T_a, w_T_b and a_Tcap_b
    // compute pose difference
    Eigen::Matrix<T, 3, 3> diff = a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

    e[0] = diff(0, 2);
    e[1] = diff(1, 2);
    e[2] = ceres::asin( NormaliseAngle(diff(1, 0)));

    return true;
  }

  static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta) {
    return (new ceres::AutoDiffCostFunction<RelativeMotionError, 3, 3, 3>(
        new RelativeMotionError(dx, dy, dtheta)));
  };

 private:
  // the observed displacement of frame b w.r.t. frame a in the x direction
  const double dx_;
  // the observed displacement of frame b w.r.t. frame a in the y direction
  const double dy_;
  // the observed orientation of frame b w.r.t. frame a
  const double dtheta_;
};

struct DCSLoopClosureError {
  // Observation for the edge
  DCSLoopClosureError(double dx, double dy, double dtheta)
      : dx_(dx), dy_(dy), dtheta_(NormaliseAngle(dtheta)) {
  }

  // calculate the error for each edge. a and b are 3-vectors representing state of the node ie. x,y,theta
  template <typename T>
  bool operator()(const T* const a, const T* const b, T* e) const {
    // Convert a to T1 ^w_T_a
    Eigen::Matrix<T, 3, 3> w_T_a = IsometricTransform2D<T>(a[0], a[1], a[2]);

    // Convert b to T2 ^w_T_a
    Eigen::Matrix<T, 3, 3> w_T_b = IsometricTransform2D<T>(b[0], b[1], b[2]);

    // Convert observed transform a_Tcap_b
    Eigen::Matrix<T, 3, 3> a_Tcap_b = IsometricTransform2D<T>(T(dx_), T(dy_), T(dtheta_));

    // now we have :: w_T_a, w_T_b and a_Tcap_b
    // compute pose difference
    Eigen::Matrix<T, 3, 3> diff = a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

    // psi - scalar (covariance term. See the paper on DCS for derivation)
    // T psi = T(1.0) / (T(1.0) + exp( T(-2.0)*s[0] ));
    // T psi = max( T(0.0), min( T(1.0), s[0] ) );

    T res = diff(0, 2) * diff(0, 2) + diff(1, 2) * diff(1, 2);  // + asin( diff(1,0) )*asin( diff(1,0) );
    // T psi_org = T(.3) * T(s_cap) / ( T(1.0) + res ) ;
    T psi_org = sqrt(T(2.0) * T(.5) / (T(.5) + res));
    // e[0] = psi ;
    // e[1] = T(0.0);
    // e[2] = T(0.0);
    // return true;
    T psi = std::min(T(1.0), psi_org);

    e[0] = psi * diff(0, 2);
    e[1] = psi * diff(1, 2);
    e[2] = psi * ceres::asin(NormaliseAngle(diff(1, 0)));

    return true;
  }

  static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta) {
    return (new ceres::AutoDiffCostFunction<DCSLoopClosureError, 3, 3, 3>(new DCSLoopClosureError(dx, dy, dtheta)));
  };

 private:
  // the observed displacement of frame b w.r.t. frame a in the x direction
  const double dx_;
  // the observed displacement of frame b w.r.t. frame a in the y direction
  const double dy_;
  // the observed orientation of frame b w.r.t. frame a
  const double dtheta_;
};

#endif  //_POSE_GRAPH_2D_ERROR_TERMS_H_
