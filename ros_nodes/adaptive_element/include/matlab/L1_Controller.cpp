//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: L1_Controller.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 09-Apr-2022 14:54:34
//

// Include Files
#include "L1_Controller.h"
#include "expm.h"
#include "inv.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// adaptive element parameters
//
// Arguments    : double adaptiveGain
//                double cutOffFrequency
// Return Type  : L1_Controller *
//
L1_Controller *L1_Controller::init(double adaptiveGain, double cutOffFrequency)
{
  static const signed char b[36]{1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  L1_Controller *b_L1_Controller;
  int i;
  b_L1_Controller = this;
  b_L1_Controller->u_L1[0] = 0.0;
  b_L1_Controller->u_L1[1] = 0.0;
  b_L1_Controller->u_L1[2] = 0.0;
  b_L1_Controller->u_L1[3] = 0.0;
  for (i = 0; i < 6; i++) {
    b_L1_Controller->z_hat[i] = 0.0;
  }
  b_L1_Controller->tau_p = 0.036889;
  b_L1_Controller->tau_q = 0.06539;
  b_L1_Controller->tau_r = 0.083315;
  b_L1_Controller->m = 1.0;
  b_L1_Controller->w_co = cutOffFrequency;
  // cut off frequency
  for (i = 0; i < 36; i++) {
    b_L1_Controller->A_s[i] = -adaptiveGain * static_cast<double>(b[i]);
  }
  // adaption gains (Hurwitz)
  b_L1_Controller->K_col = 5.0 * b_L1_Controller->m * 9.81;
  b_L1_Controller->K_phi =
      295.9283 / b_L1_Controller->tau_p * 0.017453292519943295;
  b_L1_Controller->K_theta =
      -299.1935 / b_L1_Controller->tau_q * 0.017453292519943295;
  b_L1_Controller->K_psi =
      -667.8047 / b_L1_Controller->tau_r * 0.017453292519943295;
  return b_L1_Controller;
}

//
// deconstruct state vector
//
// Arguments    : const double x[13]
//                const double u_mpc[4]
//                double dt
// Return Type  : void
//
void L1_Controller::updateController(const double x[13], const double u_mpc[4],
                                     double dt)
{
  static const signed char a[36]{-1, 0, 0,  0, 0,  0, 0, -1, 0, 0,  0, 0,
                                 0,  0, -1, 0, 0,  0, 0, 0,  0, -1, 0, 0,
                                 0,  0, 0,  0, -1, 0, 0, 0,  0, 0,  0, -1};
  static const signed char iv[12]{0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1};
  double a_tmp[36];
  double b[36];
  double b_I[36];
  double b_a_tmp[36];
  double dv[36];
  double g[24];
  double g_T[12];
  double R_bi[9];
  double b_L1_Controller[6];
  double c_a_tmp[6];
  double sigma[6];
  double z[6];
  double L1_Controller_idx_1;
  double L1_Controller_idx_2;
  double M_mpc_idx_0;
  double M_mpc_idx_1;
  double M_mpc_idx_2;
  double R_bi_tmp;
  double T_mpc;
  double b_R_bi_tmp;
  double c_R_bi_tmp;
  double d_R_bi_tmp;
  int g_tmp;
  int i;
  int i1;
  // L1-Adaptive Augmentation
  R_bi_tmp = x[3] * x[3];
  T_mpc = x[4] * x[4];
  M_mpc_idx_0 = x[5] * x[5];
  M_mpc_idx_1 = x[6] * x[6];
  R_bi[0] = ((R_bi_tmp + T_mpc) - M_mpc_idx_0) - M_mpc_idx_1;
  M_mpc_idx_2 = x[4] * x[5];
  b_R_bi_tmp = x[3] * x[6];
  R_bi[1] = 2.0 * (M_mpc_idx_2 + b_R_bi_tmp);
  c_R_bi_tmp = x[4] * x[6];
  d_R_bi_tmp = x[3] * x[5];
  R_bi[2] = 2.0 * (c_R_bi_tmp - d_R_bi_tmp);
  R_bi[3] = 2.0 * (M_mpc_idx_2 - b_R_bi_tmp);
  R_bi_tmp -= T_mpc;
  R_bi[4] = (R_bi_tmp + M_mpc_idx_0) - M_mpc_idx_1;
  T_mpc = x[5] * x[6];
  M_mpc_idx_2 = x[3] * x[4];
  R_bi[5] = 2.0 * (T_mpc + M_mpc_idx_2);
  R_bi[6] = 2.0 * (c_R_bi_tmp + d_R_bi_tmp);
  R_bi[7] = 2.0 * (T_mpc - M_mpc_idx_2);
  R_bi[8] = (R_bi_tmp - M_mpc_idx_0) + M_mpc_idx_1;
  z[0] = x[7];
  z[1] = x[8];
  z[2] = x[9];
  z[3] = x[10];
  z[4] = x[11];
  z[5] = x[12];
  // state vector
  T_mpc = K_col * u_mpc[0] / m;
  M_mpc_idx_0 = -1.0 / tau_p * x[10] + K_phi * u_mpc[1];
  M_mpc_idx_1 = -1.0 / tau_q * x[11] + K_theta * u_mpc[2];
  M_mpc_idx_2 = -1.0 / tau_r * x[12] + K_psi * u_mpc[3];
  // desired dynamics
  for (i = 0; i < 3; i++) {
    g[i] = R_bi[i + 6];
    g_tmp = 6 * (i + 1);
    g[g_tmp] = 0.0;
    g[g_tmp + 1] = 0.0;
    g[g_tmp + 2] = 0.0;
  }
  for (i = 0; i < 4; i++) {
    g[6 * i + 3] = iv[3 * i];
    g[6 * i + 4] = iv[3 * i + 1];
    g[6 * i + 5] = iv[3 * i + 2];
  }
  // uncertainty in matched component
  g_T[0] = R_bi[0];
  g_T[6] = R_bi[3];
  g_T[1] = R_bi[1];
  g_T[7] = R_bi[4];
  g_T[2] = R_bi[2];
  g_T[8] = R_bi[5];
  for (i = 0; i < 2; i++) {
    g_T[6 * i + 3] = 0.0;
    g_T[6 * i + 4] = 0.0;
    g_T[6 * i + 5] = 0.0;
  }
  // uncertainty in unmatched dynamics
  for (i = 0; i < 36; i++) {
    a_tmp[i] = A_s[i];
    b_I[i] = 0.0;
  }
  for (g_tmp = 0; g_tmp < 6; g_tmp++) {
    b_I[g_tmp + 6 * g_tmp] = 1.0;
  }
  for (i = 0; i < 36; i++) {
    b_a_tmp[i] = a_tmp[i] * dt;
  }
  coder::expm(b_a_tmp, b);
  for (i = 0; i < 36; i++) {
    b[i] -= b_I[i];
    b_a_tmp[i] = a_tmp[i] * dt;
  }
  coder::expm(b_a_tmp, a_tmp);
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      g_tmp = i1 + 6 * i;
      b_a_tmp[g_tmp] = g[g_tmp];
    }
  }
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_a_tmp[i1 + 6 * (i + 4)] = g_T[i1 + 6 * i];
    }
  }
  coder::inv(b_a_tmp, b_I);
  coder::inv(A_s, dv);
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp = 0.0;
      for (g_tmp = 0; g_tmp < 6; g_tmp++) {
        b_R_bi_tmp += dv[i + 6 * g_tmp] * b[g_tmp + 6 * i1];
      }
      b_a_tmp[i + 6 * i1] = b_R_bi_tmp;
    }
  }
  coder::inv(b_a_tmp, dv);
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp = 0.0;
      for (g_tmp = 0; g_tmp < 6; g_tmp++) {
        b_R_bi_tmp +=
            static_cast<double>(a[i + 6 * g_tmp]) * b_I[g_tmp + 6 * i1];
      }
      b[i + 6 * i1] = b_R_bi_tmp;
    }
    b_L1_Controller[i] = z_hat[i] - z[i];
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp = 0.0;
      for (g_tmp = 0; g_tmp < 6; g_tmp++) {
        b_R_bi_tmp += b[i + 6 * g_tmp] * dv[g_tmp + 6 * i1];
      }
      b_a_tmp[i + 6 * i1] = b_R_bi_tmp;
    }
  }
  for (i = 0; i < 6; i++) {
    b_R_bi_tmp = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp += a_tmp[i + 6 * i1] * b_L1_Controller[i1];
    }
    c_a_tmp[i] = b_R_bi_tmp;
  }
  for (i = 0; i < 6; i++) {
    b_R_bi_tmp = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp += b_a_tmp[i + 6 * i1] * c_a_tmp[i1];
    }
    sigma[i] = b_R_bi_tmp;
  }
  // piecewise-constant adaptation law
  //  LPF on the adaption
  R_bi_tmp = std::exp(-w_co * dt);
  u_L1[0] = u_L1[0] * R_bi_tmp - sigma[0] * (1.0 - R_bi_tmp);
  d_R_bi_tmp = u_L1[0] + sigma[0];
  u_L1[1] = u_L1[1] * R_bi_tmp - sigma[1] * (1.0 - R_bi_tmp);
  L1_Controller_idx_1 = u_L1[1] + sigma[1];
  u_L1[2] = u_L1[2] * R_bi_tmp - sigma[2] * (1.0 - R_bi_tmp);
  L1_Controller_idx_2 = u_L1[2] + sigma[2];
  u_L1[3] = u_L1[3] * R_bi_tmp - sigma[3] * (1.0 - R_bi_tmp);
  c_R_bi_tmp = u_L1[3] + sigma[3];
  b_L1_Controller[0] = T_mpc * R_bi[6];
  b_L1_Controller[3] = M_mpc_idx_0;
  b_L1_Controller[1] = T_mpc * R_bi[7];
  b_L1_Controller[4] = M_mpc_idx_1;
  b_L1_Controller[2] = T_mpc * R_bi[8] + 9.81;
  b_L1_Controller[5] = M_mpc_idx_2;
  b_R_bi_tmp = sigma[4];
  R_bi_tmp = sigma[5];
  for (i = 0; i < 6; i++) {
    z[i] = z_hat[i] - z[i];
    c_a_tmp[i] = (b_L1_Controller[i] +
                  (((g[i] * d_R_bi_tmp + g[i + 6] * L1_Controller_idx_1) +
                    g[i + 12] * L1_Controller_idx_2) +
                   g[i + 18] * c_R_bi_tmp)) +
                 (g_T[i] * b_R_bi_tmp + g_T[i + 6] * R_bi_tmp);
  }
  for (i = 0; i < 6; i++) {
    b_R_bi_tmp = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      b_R_bi_tmp += A_s[i + 6 * i1] * z[i1];
    }
    z_hat[i] += (c_a_tmp[i] + b_R_bi_tmp) * dt;
  }
  u[0] = u_mpc[0] + u_L1[0] / K_col;
  u[1] = u_mpc[1] + u_L1[1] / K_phi;
  u[2] = u_mpc[2] + u_L1[2] / K_theta;
  u[3] = u_mpc[3] + u_L1[3] / K_psi;
}

//
// File trailer for L1_Controller.cpp
//
// [EOF]
//
