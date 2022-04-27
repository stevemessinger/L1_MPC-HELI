//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: foo.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 27-Apr-2022 11:15:15
//

// Include Files
#include "foo.h"
#include "L1_Controller.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const double x[13]
//                const double u_mpc[4]
//                double dt
//                double adaptiveGain
//                double cutoffFrequency
//                double u[4]
//                double a[6]
//                double *b
//                double u2[4]
//                double a2[6]
//                double *b2
// Return Type  : void
//
void foo(const double x[13], const double u_mpc[4], double dt, double
         adaptiveGain, double cutoffFrequency, double u[4], double a[6], double *
         b, double u2[4], double a2[6], double *b2)
{
  L1_Controller L1_cont;
  L1_Controller L1_cont2;
  int i;
  L1_cont.init(adaptiveGain, cutoffFrequency);
  L1_cont.updateController(x, u_mpc, dt);
  u[0] = L1_cont.u[0];
  u[1] = L1_cont.u[1];
  u[2] = L1_cont.u[2];
  u[3] = L1_cont.u[3];
  for (i = 0; i < 6; i++) {
    a[i] = L1_cont.z_hat[i];
  }

  *b = L1_cont.w_co;
  L1_cont2.init(adaptiveGain, cutoffFrequency);
  L1_cont2.updateController(x, u_mpc, dt);
  u2[0] = L1_cont2.u[0];
  u2[1] = L1_cont2.u[1];
  u2[2] = L1_cont2.u[2];
  u2[3] = L1_cont2.u[3];
  for (i = 0; i < 6; i++) {
    a2[i] = L1_cont2.z_hat[i];
  }

  *b2 = L1_cont2.w_co;
}

//
// File trailer for foo.cpp
//
// [EOF]
//
