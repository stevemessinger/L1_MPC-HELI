//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EKF.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 27-Apr-2022 13:20:00
//
#ifndef EKF_H
#define EKF_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class EKF
{
 public:
  EKF *init(const double x0[16]);
  void calc_estimate(const double y[7], const double imu[6], double dt);
  static void derivative(const double x[16], const double imu[6], double xdot[16]);
  double P[256];
  double Q[256];
  double R[49];
  double x_hat[16];
  double cr[49];
  double r[7];
};

#endif

//
// File trailer for EKF.h
//
// [EOF]
//
