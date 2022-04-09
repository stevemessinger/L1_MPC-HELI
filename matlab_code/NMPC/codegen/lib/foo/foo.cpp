//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: foo.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 09-Apr-2022 14:02:55
//

// Include Files
#include "foo.h"
#include "L1_Controller.h"
#include "expm.h"
#include "inv.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[13]
//                const double u_mpc[4]
//                double dt
//                double u[4]
// Return Type  : void
//
void foo(const double x[13], const double u_mpc[4], double dt, double u[4])
{
  static const signed char b_a[36]{-1, 0, 0,  0, 0,  0, 0, -1, 0, 0,  0, 0,
                                   0,  0, -1, 0, 0,  0, 0, 0,  0, -1, 0, 0,
                                   0,  0, 0,  0, -1, 0, 0, 0,  0, 0,  0, -1};
  static const signed char y[36]{-5, 0, 0,  0, 0,  0, 0, -5, 0, 0,  0, 0,
                                 0,  0, -5, 0, 0,  0, 0, 0,  0, -5, 0, 0,
                                 0,  0, 0,  0, -5, 0, 0, 0,  0, 0,  0, -5};
  static const signed char iv[12]{0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
  L1_Controller L1_cont;
  double b[36];
  double b_I[36];
  double b_L1_cont[36];
  double dv[36];
  double dv1[36];
  double R_bi[9];
  double a[6];
  double c_I[6];
  double R_bi_tmp;
  double b_R_bi_tmp;
  double c_R_bi_tmp;
  double d_R_bi_tmp;
  double e_R_bi_tmp;
  double f_R_bi_tmp;
  double g_R_bi_tmp;
  double h_R_bi_tmp;
  int i;
  int i1;
  int k;
  //  adaptive element parameters
  // cut off frequency
  // adaption gains (Hurwitz)
  // deconstruct state vector
  // L1-Adaptive Augmentation
  R_bi_tmp = x[3] * x[3];
  b_R_bi_tmp = x[4] * x[4];
  c_R_bi_tmp = x[5] * x[5];
  d_R_bi_tmp = x[6] * x[6];
  R_bi[0] = ((R_bi_tmp + b_R_bi_tmp) - c_R_bi_tmp) - d_R_bi_tmp;
  e_R_bi_tmp = x[4] * x[5];
  f_R_bi_tmp = x[3] * x[6];
  R_bi[1] = 2.0 * (e_R_bi_tmp + f_R_bi_tmp);
  g_R_bi_tmp = x[4] * x[6];
  h_R_bi_tmp = x[3] * x[5];
  R_bi[2] = 2.0 * (g_R_bi_tmp - h_R_bi_tmp);
  R_bi[3] = 2.0 * (e_R_bi_tmp - f_R_bi_tmp);
  R_bi_tmp -= b_R_bi_tmp;
  R_bi[4] = (R_bi_tmp + c_R_bi_tmp) - d_R_bi_tmp;
  b_R_bi_tmp = x[5] * x[6];
  e_R_bi_tmp = x[3] * x[4];
  R_bi[5] = 2.0 * (b_R_bi_tmp + e_R_bi_tmp);
  R_bi[6] = 2.0 * (g_R_bi_tmp + h_R_bi_tmp);
  R_bi[7] = 2.0 * (b_R_bi_tmp - e_R_bi_tmp);
  R_bi[8] = (R_bi_tmp - c_R_bi_tmp) + d_R_bi_tmp;
  // state vector
  // desired dynamics
  // uncertainty in matched component
  // uncertainty in unmatched dynamics
  for (i = 0; i < 36; i++) {
    L1_cont.A_s[i] = y[i];
    b_I[i] = 0.0;
  }
  for (k = 0; k < 6; k++) {
    b_I[k + 6 * k] = 1.0;
  }
  for (i = 0; i < 36; i++) {
    b_L1_cont[i] = L1_cont.A_s[i] * dt;
  }
  coder::expm(b_L1_cont, b);
  for (i = 0; i < 36; i++) {
    b[i] -= b_I[i];
    b_L1_cont[i] = L1_cont.A_s[i] * dt;
  }
  coder::expm(b_L1_cont, b_I);
  // piecewise-constant adaptation law
  //  LPF on the adaption
  R_bi_tmp = std::exp(-15.0 * dt);
  for (i = 0; i < 3; i++) {
    b_L1_cont[i] = R_bi[i + 6];
    k = 6 * (i + 1);
    b_L1_cont[k] = 0.0;
    b_L1_cont[k + 1] = 0.0;
    b_L1_cont[k + 2] = 0.0;
  }
  for (i = 0; i < 4; i++) {
    b_L1_cont[6 * i + 3] = iv[3 * i];
    b_L1_cont[6 * i + 4] = iv[3 * i + 1];
    b_L1_cont[6 * i + 5] = iv[3 * i + 2];
  }
  b_L1_cont[24] = R_bi[0];
  b_L1_cont[30] = R_bi[3];
  b_L1_cont[25] = R_bi[1];
  b_L1_cont[31] = R_bi[4];
  b_L1_cont[26] = R_bi[2];
  b_L1_cont[32] = R_bi[5];
  for (i = 0; i < 2; i++) {
    k = 6 * (i + 4);
    b_L1_cont[k + 3] = 0.0;
    b_L1_cont[k + 4] = 0.0;
    b_L1_cont[k + 5] = 0.0;
  }
  coder::inv(b_L1_cont, dv);
  coder::inv(L1_cont.A_s, dv1);
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp = 0.0;
      for (k = 0; k < 6; k++) {
        b_R_bi_tmp += dv1[i + 6 * k] * b[k + 6 * i1];
      }
      b_L1_cont[i + 6 * i1] = b_R_bi_tmp;
    }
  }
  coder::inv(b_L1_cont, dv1);
  a[0] = 0.0 - x[7];
  a[1] = 0.0 - x[8];
  a[2] = 0.0 - x[9];
  a[3] = 0.0 - x[10];
  a[4] = 0.0 - x[11];
  a[5] = 0.0 - x[12];
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp = 0.0;
      for (k = 0; k < 6; k++) {
        b_R_bi_tmp += static_cast<double>(b_a[i + 6 * k]) * dv[k + 6 * i1];
      }
      b[i + 6 * i1] = b_R_bi_tmp;
    }
    c_I[i] = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp = 0.0;
      for (k = 0; k < 6; k++) {
        b_R_bi_tmp += b[i + 6 * k] * dv1[k + 6 * i1];
      }
      k = i + 6 * i1;
      b_L1_cont[k] = b_R_bi_tmp;
      c_I[i] += b_I[k] * a[i1];
    }
  }
  for (i = 0; i < 6; i++) {
    b_R_bi_tmp = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp += b_L1_cont[i + 6 * i1] * c_I[i1];
    }
    a[i] = b_R_bi_tmp;
  }
  u[0] = u_mpc[0] +
         (0.0 * R_bi_tmp - a[0] * (1.0 - R_bi_tmp)) / 49.050000000000004;
  u[1] = u_mpc[1] +
         (0.0 * R_bi_tmp - a[1] * (1.0 - R_bi_tmp)) / 140.01255617743868;
  u[2] = u_mpc[2] +
         (0.0 * R_bi_tmp - a[2] * (1.0 - R_bi_tmp)) / -79.857954971182949;
  u[3] = u_mpc[3] +
         (0.0 * R_bi_tmp - a[3] * (1.0 - R_bi_tmp)) / -139.89546630610306;
}

//
// File trailer for foo.cpp
//
// [EOF]
//
