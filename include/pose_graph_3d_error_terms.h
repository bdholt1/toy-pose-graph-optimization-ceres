#ifndef _POSE_GRAPH_3D_ERROR_TERMS_H_
#define _POSE_GRAPH_3D_ERROR_TERMS_H_

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <Eigen/Eigen>



struct RelativeMotionError {
  RelativeMotionError(Eigen::Vector3d p, Eigen::Quaterniond q, Eigen::Matrix<double, 6, 6> sqrt_information)
      : measured_p_(p), measured_q_(q), sqrt_information_(sqrt_information) {}

  // calculate the error for each edge. a and b are 3-vectors representing state
  // of the node ie. x,y,theta
  template <typename T>
  bool operator()(const T* const p_a_ptr, const T* const q_a_ptr, const T* const p_b_ptr, const T* const q_b_ptr,  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_a(p_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T> > q_a(q_a_ptr);

    Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_b(p_b_ptr);
    Eigen::Map<const Eigen::Quaternion<T> > q_b(q_b_ptr);

    //Compute the relative transformation between frame a and frame b
    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<T> delta_q = measured_q_.template cast<T>() * q_ab_estimated.conjugate();

    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        p_ab_estimated - measured_p_.template cast<T>();
    residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector3d& p,
      const Eigen::Quaterniond& q,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<RelativeMotionError, 6, 3, 4, 3, 4>(
        new RelativeMotionError(p, q, sqrt_information));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // the measured displacement of frame b w.r.t. frame a in frame a
  const Eigen::Vector3d measured_p_;
  // the measured rotation of frame b w.r.t. frame a in frame a
  const Eigen::Quaterniond measured_q_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};


#endif //_POSE_GRAPH_3D_ERROR_TERMS_H_
