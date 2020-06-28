#ifndef _NORMALISE_ANGLE_H_
#define _NORMALISE_ANGLE_H_

#include <ceres/ceres.h>

template <typename T>
T NormaliseAngle(const T& angle_radians) {
  // Use ceres::floor because it is specialised for double and Jet types
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

#endif  //_NORMALISE_ANGLE_H_
