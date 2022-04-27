//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: L1_Controller.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 27-Apr-2022 11:15:15
//
#ifndef L1_CONTROLLER_H
#define L1_CONTROLLER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class L1_Controller
{
 public:
  L1_Controller *init(double adaptiveGain, double cutOffFrequency);
  void updateController(const double x[13], const double u_mpc[4], double dt);
  double u_L1[4];
  double u[4];
  double z_hat[6];
  double w_co;
  double A_s[36];
  double tau_p;
  double tau_q;
  double tau_r;
  double K_col;
  double K_phi;
  double K_theta;
  double K_psi;
};

#endif

//
// File trailer for L1_Controller.h
//
// [EOF]
//
