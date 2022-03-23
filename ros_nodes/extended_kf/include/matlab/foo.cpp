//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: foo.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 21-Mar-2022 13:01:25
//

// Include Files
#include "foo.h"
#include "EKF.h"
#include <cstring>

// Function Definitions
//
// initialize EKF
// Arguments    : const double x0[16]
//                double dt
//                const double y[7]
//                const double imu[6]
//                double xhat[16]
//                double P[256]
// Return Type  : void
//
void foo(const double x0[16], double dt, const double y[7], const double imu[6],
         double xhat[16], double P[256])
{
  EKF EKF_C;
  EKF_C.init(x0);
  EKF_C.calc_estimate(y, imu, dt);

  // save some variables for plotting
  std::memcpy(&xhat[0], &EKF_C.x_hat[0], 16U * sizeof(double));
  std::memcpy(&P[0], &EKF_C.P[0], 256U * sizeof(double));
}

//
// File trailer for foo.cpp
//
// [EOF]
//
