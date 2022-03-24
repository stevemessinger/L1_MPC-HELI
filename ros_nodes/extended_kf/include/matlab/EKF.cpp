//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EKF.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 21-Mar-2022 13:01:25
//

// Include Files
#include "EKF.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// EKF process model from IMU data
//  state: [1:p_x 2:p_y 3:p_z 4:v_x 5:v_y 6:v_z 7:q0 8:q1 9:q2 10:q3 11:b_ax 12:b_ay 13:b_az 14:b_wx 15:b_wy 16:b_wz]
//  imu: [ax ay az wx wy wz]
// Arguments    : const double x[16]
//                const double imu[6]
//                double xdot[16]
// Return Type  : void
//
void EKF::derivative(const double x[16], const double imu[6], double xdot[16])
{
  double dv1[16];
  double dv[9];
  double d;
  double d1;
  double imu_idx_0;
  double imu_idx_1;
  double imu_idx_2;
  double x_idx_3;
  int i;

  // initialize derivative
  // assign temp variables
  // quaternions
  // linear velocities
  // linear bias terms
  // angular bias terms
  // imu estimates [xyz_accel pqr]
  // DCM body to inertial frame
  xdot[0] = x[3];
  xdot[1] = x[4];
  xdot[2] = x[5];
  imu_idx_0 = x[9] * x[9];
  imu_idx_1 = x[8] * x[8];
  dv[0] = 1.0 - 2.0 * (imu_idx_1 + imu_idx_0);
  imu_idx_2 = x[7] * x[8];
  x_idx_3 = x[6] * x[9];
  dv[3] = 2.0 * (imu_idx_2 - x_idx_3);
  d = x[7] * x[9];
  d1 = x[6] * x[8];
  dv[6] = 2.0 * (d + d1);
  dv[1] = 2.0 * (imu_idx_2 + x_idx_3);
  imu_idx_2 = x[7] * x[7];
  dv[4] = 1.0 - 2.0 * (imu_idx_2 + imu_idx_0);
  imu_idx_0 = 2.0 * (x[8] * x[9] - x[6] * x[7]);
  dv[7] = imu_idx_0;
  dv[2] = 2.0 * (d - d1);
  dv[5] = imu_idx_0;
  dv[8] = 1.0 - 2.0 * (imu_idx_2 + imu_idx_1);
  imu_idx_0 = imu[0] - x[10];
  imu_idx_1 = imu[1] - x[11];
  imu_idx_2 = imu[2] - x[12];
  for (i = 0; i < 3; i++) {
    xdot[i + 3] = (dv[i] * imu_idx_0 + dv[i + 3] * imu_idx_1) + dv[i + 6] *
      imu_idx_2;
  }

  //  - [0;0;9.81]; % linear accelerations
  dv1[0] = 0.0;
  imu_idx_0 = imu[3] * 0.017453292519943295 - x[13];
  imu_idx_1 = 0.5 * -imu_idx_0;
  dv1[4] = imu_idx_1;
  imu_idx_2 = imu[4] * 0.017453292519943295 - x[14];
  x_idx_3 = 0.5 * -imu_idx_2;
  dv1[8] = x_idx_3;
  d = imu[5] * 0.017453292519943295 - x[15];
  d1 = 0.5 * -d;
  dv1[12] = d1;
  imu_idx_0 *= 0.5;
  dv1[1] = imu_idx_0;
  dv1[5] = 0.0;
  d *= 0.5;
  dv1[9] = d;
  dv1[13] = x_idx_3;
  imu_idx_2 *= 0.5;
  dv1[2] = imu_idx_2;
  dv1[6] = d1;
  dv1[10] = 0.0;
  dv1[14] = imu_idx_0;
  dv1[3] = d;
  dv1[7] = imu_idx_2;
  dv1[11] = imu_idx_1;
  dv1[15] = 0.0;
  imu_idx_0 = x[6];
  imu_idx_1 = x[7];
  imu_idx_2 = x[8];
  x_idx_3 = x[9];
  for (i = 0; i < 4; i++) {
    xdot[i + 6] = ((dv1[i] * imu_idx_0 + dv1[i + 4] * imu_idx_1) + dv1[i + 8] *
                   imu_idx_2) + dv1[i + 12] * x_idx_3;
  }

  //  quaternion dot equations
  for (i = 0; i < 6; i++) {
    xdot[i + 10] = 0.0;
  }

  // bias terms -> derivative = 0
}

//
// calc_estimate -> updates EKF state estimate given y(sensor
// measurement) and dt(EKF frequency)
//  EKF Process Model
//  integrate EKF equations using RK4
// Arguments    : const double y[7]
//                const double imu[6]
//                double dt
// Return Type  : void
//
void EKF::calc_estimate(const double y[7], const double imu[6], double dt)
{
  static const signed char c_a[112] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const signed char iv3[112] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 };

  static const signed char iv[16] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0 };

  static const signed char iv1[16] = { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0 };

  static const signed char iv2[16] = { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0 };

  double A[256];
  double Pdot1[256];
  double Pdot2[256];
  double Pdot2_tmp[256];
  double Pdot3[256];
  double b_A[256];
  double c_EKF[256];
  double K[112];
  double b_a[112];
  double b[49];
  double x[49];
  double b_EKF[16];
  double xdot1[16];
  double xdot2[16];
  double xdot3[16];
  double xdot4[16];
  double q0;
  double q1;
  int a;
  int b_i;
  int b_tmp;
  int i;
  int i1;
  int ix;
  int iy;
  int j;
  int jp1j;
  int k;
  signed char ipiv[7];
  signed char p[7];
  EKF::derivative((this->x_hat), (imu), (xdot1));
  for (i = 0; i < 16; i++) {
    b_EKF[i] = this->x_hat[i] + xdot1[i] * dt / 2.0;
  }

  EKF::derivative((b_EKF), (imu), (xdot2));
  for (i = 0; i < 16; i++) {
    b_EKF[i] = this->x_hat[i] + xdot2[i] * dt / 2.0;
  }

  EKF::derivative((b_EKF), (imu), (xdot3));
  for (i = 0; i < 16; i++) {
    b_EKF[i] = this->x_hat[i] + xdot3[i] * dt;
  }

  EKF::derivative((b_EKF), (imu), (xdot4));
  for (i = 0; i < 16; i++) {
    this->x_hat[i] += (((xdot1[i] + 2.0 * xdot2[i]) + 2.0 * xdot3[i]) + xdot4[i])
      / 6.0 * dt;
  }

  double A_tmp;
  double b_A_tmp;
  double b_wx;
  double b_wy;
  double b_wz;
  double c_A_tmp;
  double d_A_tmp;
  double e_A_tmp;
  double f_A_tmp;
  double g_A_tmp;
  double h_A_tmp;
  double i_A_tmp;
  double j_A_tmp;
  double k_A_tmp;
  double l_A_tmp;
  double m_A_tmp;
  double n_A_tmp;
  double o_A_tmp;
  double p_A_tmp;
  double q2;
  double q3;
  double q_A_tmp;

  //  asign temp variables
  q0 = this->x_hat[6];
  q1 = this->x_hat[7];
  q2 = this->x_hat[8];
  q3 = this->x_hat[9];
  b_wx = this->x_hat[13];
  b_wy = this->x_hat[14];
  b_wz = this->x_hat[15];

  // imu estimates [xyz_accel pqr]
  //  calculate A matrix
  A[3] = 0.0;
  A[19] = 0.0;
  A[35] = 0.0;
  A[51] = 0.0;
  A[67] = 0.0;
  A[83] = 0.0;
  A_tmp = imu[1] - this->x_hat[11];
  b_A_tmp = imu[2] - this->x_hat[12];
  c_A_tmp = 2.0 * q2 * b_A_tmp;
  A[99] = -2.0 * q3 * A_tmp + c_A_tmp;
  d_A_tmp = 2.0 * q3 * b_A_tmp;
  e_A_tmp = 2.0 * q2 * A_tmp;
  A[115] = e_A_tmp + d_A_tmp;
  f_A_tmp = imu[0] - this->x_hat[10];
  g_A_tmp = 2.0 * q0 * b_A_tmp;
  h_A_tmp = 2.0 * q1 * A_tmp;
  A[131] = (-4.0 * q2 * f_A_tmp + h_A_tmp) + g_A_tmp;
  i_A_tmp = 2.0 * q1 * b_A_tmp;
  j_A_tmp = 2.0 * q0 * A_tmp;
  A[147] = (-4.0 * q3 * f_A_tmp - j_A_tmp) + i_A_tmp;
  k_A_tmp = q3 * q3;
  l_A_tmp = q2 * q2;
  A[163] = 2.0 * (l_A_tmp + k_A_tmp) + -1.0;
  m_A_tmp = q1 * q2;
  n_A_tmp = q0 * q3;
  A[179] = -2.0 * (m_A_tmp - n_A_tmp);
  o_A_tmp = q1 * q3;
  p_A_tmp = q0 * q2;
  A[195] = -2.0 * (o_A_tmp + p_A_tmp);
  A[211] = 0.0;
  A[227] = 0.0;
  A[243] = 0.0;
  A[4] = 0.0;
  A[20] = 0.0;
  A[36] = 0.0;
  A[52] = 0.0;
  A[68] = 0.0;
  A[84] = 0.0;
  q_A_tmp = 2.0 * q3 * f_A_tmp;
  A[100] = q_A_tmp - i_A_tmp;
  A[116] = (2.0 * q2 * f_A_tmp - 4.0 * q1 * A_tmp) - g_A_tmp;
  g_A_tmp = 2.0 * q1 * f_A_tmp;
  A[132] = g_A_tmp + d_A_tmp;
  A[148] = (2.0 * q0 * f_A_tmp - 4.0 * q3 * A_tmp) + c_A_tmp;
  A[164] = -2.0 * (m_A_tmp + n_A_tmp);
  c_A_tmp = q1 * q1;
  A[180] = 2.0 * (c_A_tmp + k_A_tmp) + -1.0;
  d_A_tmp = -2.0 * (q2 * q3 - q0 * q1);
  A[196] = d_A_tmp;
  A[212] = 0.0;
  A[228] = 0.0;
  A[244] = 0.0;
  A[5] = 0.0;
  A[21] = 0.0;
  A[37] = 0.0;
  A[53] = 0.0;
  A[69] = 0.0;
  A[85] = 0.0;
  A[101] = -2.0 * q2 * f_A_tmp - h_A_tmp;
  A[117] = (q_A_tmp - j_A_tmp) - 4.0 * q1 * b_A_tmp;
  A[133] = (-2.0 * q0 * f_A_tmp + 2.0 * q3 * A_tmp) - 4.0 * q2 * b_A_tmp;
  A[149] = g_A_tmp + e_A_tmp;
  A[165] = -2.0 * (o_A_tmp - p_A_tmp);
  A[181] = d_A_tmp;
  A[197] = 2.0 * (c_A_tmp + l_A_tmp) + -1.0;
  A[213] = 0.0;
  A[229] = 0.0;
  A[245] = 0.0;
  A[6] = 0.0;
  A[22] = 0.0;
  A[38] = 0.0;
  A[54] = 0.0;
  A[70] = 0.0;
  A[86] = 0.0;
  A[102] = 0.0;
  A_tmp = 0.5 * (-imu[3] + b_wx);
  A[118] = A_tmp;
  b_A_tmp = 0.5 * (-imu[4] + b_wy);
  A[134] = b_A_tmp;
  c_A_tmp = 0.5 * (-imu[5] + b_wz);
  A[150] = c_A_tmp;
  A[166] = 0.0;
  A[182] = 0.0;
  A[198] = 0.0;
  A[214] = 0.5 * q1;
  A[230] = 0.5 * q2;
  A[246] = 0.5 * q3;
  A[7] = 0.0;
  A[23] = 0.0;
  A[39] = 0.0;
  A[55] = 0.0;
  A[71] = 0.0;
  A[87] = 0.0;
  d_A_tmp = 0.5 * (imu[3] - b_wx);
  A[103] = d_A_tmp;
  A[119] = 0.0;
  e_A_tmp = 0.5 * (imu[5] - b_wz);
  A[135] = e_A_tmp;
  A[151] = b_A_tmp;
  A[167] = 0.0;
  A[183] = 0.0;
  A[199] = 0.0;
  A[215] = -0.5 * q0;
  A[231] = 0.5 * q3;
  A[247] = -0.5 * q2;
  A[8] = 0.0;
  A[24] = 0.0;
  A[40] = 0.0;
  A[56] = 0.0;
  A[72] = 0.0;
  A[88] = 0.0;
  b_A_tmp = 0.5 * (imu[4] - b_wy);
  A[104] = b_A_tmp;
  A[120] = c_A_tmp;
  A[136] = 0.0;
  A[152] = d_A_tmp;
  A[168] = 0.0;
  A[184] = 0.0;
  A[200] = 0.0;
  A[216] = -0.5 * q3;
  A[232] = -0.5 * q0;
  A[248] = 0.5 * q1;
  A[9] = 0.0;
  A[25] = 0.0;
  A[41] = 0.0;
  A[57] = 0.0;
  A[73] = 0.0;
  A[89] = 0.0;
  A[105] = e_A_tmp;
  A[121] = b_A_tmp;
  A[137] = A_tmp;
  A[153] = 0.0;
  A[169] = 0.0;
  A[185] = 0.0;
  A[201] = 0.0;
  A[217] = 0.5 * q2;
  A[233] = -0.5 * q1;
  A[249] = -0.5 * q0;
  for (i = 0; i < 16; i++) {
    iy = i << 4;
    A[iy] = iv[i];
    A[iy + 1] = iv1[i];
    A[iy + 2] = iv2[i];
    A[iy + 10] = 0.0;
    A[iy + 11] = 0.0;
    A[iy + 12] = 0.0;
    A[iy + 13] = 0.0;
    A[iy + 14] = 0.0;
    A[iy + 15] = 0.0;
  }

  //  integrate covariance matrix P using RK4
  for (i = 0; i < 16; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      q0 = 0.0;
      q1 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        iy = jp1j << 4;
        a = i + iy;
        q0 += A[a] * this->P[jp1j + (i1 << 4)];
        q1 += this->P[a] * A[i1 + iy];
      }

      iy = i + (i1 << 4);
      c_EKF[iy] = q1;
      Pdot1[iy] = q0;
    }
  }

  for (i = 0; i < 256; i++) {
    q0 = (Pdot1[i] + c_EKF[i]) + this->Q[i];
    Pdot1[i] = q0;
    c_EKF[i] = this->P[i] + q0 * dt / 2.0;
  }

  for (i = 0; i < 16; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      q0 = 0.0;
      q1 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        iy = jp1j << 4;
        a = i + iy;
        q0 += A[a] * c_EKF[jp1j + (i1 << 4)];
        q1 += c_EKF[a] * A[i1 + iy];
      }

      iy = i + (i1 << 4);
      Pdot2_tmp[iy] = q1;
      Pdot2[iy] = q0;
    }
  }

  for (i = 0; i < 256; i++) {
    q0 = (Pdot2[i] + Pdot2_tmp[i]) + this->Q[i];
    Pdot2[i] = q0;
    c_EKF[i] = this->P[i] + q0 * dt / 2.0;
  }

  for (i = 0; i < 16; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      q0 = 0.0;
      q1 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        iy = jp1j << 4;
        a = i + iy;
        q0 += A[a] * c_EKF[jp1j + (i1 << 4)];
        q1 += c_EKF[a] * A[i1 + iy];
      }

      iy = i + (i1 << 4);
      Pdot2_tmp[iy] = q1;
      Pdot3[iy] = q0;
    }
  }

  for (i = 0; i < 256; i++) {
    q0 = (Pdot3[i] + Pdot2_tmp[i]) + this->Q[i];
    Pdot3[i] = q0;
    c_EKF[i] = this->P[i] + q0 * dt;
  }

  for (i = 0; i < 16; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      q0 = 0.0;
      q1 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        iy = jp1j << 4;
        a = i + iy;
        q0 += A[a] * c_EKF[jp1j + (i1 << 4)];
        q1 += c_EKF[a] * A[i1 + iy];
      }

      iy = i + (i1 << 4);
      Pdot2_tmp[iy] = q1;
      b_A[iy] = q0;
    }
  }

  for (i = 0; i < 256; i++) {
    this->P[i] += (((Pdot1[i] + 2.0 * Pdot2[i]) + 2.0 * Pdot3[i]) + ((b_A[i] +
      Pdot2_tmp[i]) + this->Q[i])) / 6.0 * dt;
  }

  //  EKF measurment model
  //  if we get a sensor measurement update the EKF
  //  calculate C matrix
  // make sensor estimate of measurement using x_hat
  std::memcpy(&xdot1[0], &this->x_hat[0], 16U * sizeof(double));
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      q0 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        q0 += static_cast<double>(c_a[i + 7 * jp1j]) * this->P[jp1j + (i1 << 4)];
      }

      b_a[i + 7 * i1] = q0;
    }

    for (i1 = 0; i1 < 7; i1++) {
      q0 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        q0 += b_a[i + 7 * jp1j] * static_cast<double>(iv3[jp1j + (i1 << 4)]);
      }

      jp1j = i + 7 * i1;
      this->cr[jp1j] = q0 + this->R[jp1j];
    }

    q0 = 0.0;
    for (i1 = 0; i1 < 16; i1++) {
      q0 += static_cast<double>(c_a[i + 7 * i1]) * xdot1[i1];
    }

    this->r[i] = y[i] - q0;
  }

  //  update hybrid EKF
  std::memset(&b[0], 0, 49U * sizeof(double));
  for (i = 0; i < 7; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      q0 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        q0 += static_cast<double>(c_a[i + 7 * jp1j]) * this->P[jp1j + (i1 << 4)];
      }

      b_a[i + 7 * i1] = q0;
    }

    for (i1 = 0; i1 < 7; i1++) {
      q0 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        q0 += b_a[i + 7 * jp1j] * static_cast<double>(iv3[jp1j + (i1 << 4)]);
      }

      iy = i + 7 * i1;
      x[iy] = q0 + this->R[iy];
    }

    ipiv[i] = static_cast<signed char>(i + 1);
  }

  for (j = 0; j < 6; j++) {
    int mmj_tmp;
    mmj_tmp = 5 - j;
    b_tmp = j << 3;
    jp1j = b_tmp + 2;
    iy = 7 - j;
    a = 0;
    ix = b_tmp;
    q0 = std::abs(x[b_tmp]);
    for (k = 2; k <= iy; k++) {
      ix++;
      q1 = std::abs(x[ix]);
      if (q1 > q0) {
        a = k - 1;
        q0 = q1;
      }
    }

    if (x[b_tmp + a] != 0.0) {
      if (a != 0) {
        iy = j + a;
        ipiv[j] = static_cast<signed char>(iy + 1);
        ix = j;
        for (k = 0; k < 7; k++) {
          q0 = x[ix];
          x[ix] = x[iy];
          x[iy] = q0;
          ix += 7;
          iy += 7;
        }
      }

      i = (b_tmp - j) + 7;
      for (b_i = jp1j; b_i <= i; b_i++) {
        x[b_i - 1] /= x[b_tmp];
      }
    }

    iy = b_tmp + 7;
    jp1j = b_tmp;
    for (a = 0; a <= mmj_tmp; a++) {
      q0 = x[iy];
      if (x[iy] != 0.0) {
        ix = b_tmp + 1;
        i = jp1j + 9;
        i1 = (jp1j - j) + 14;
        for (b_i = i; b_i <= i1; b_i++) {
          x[b_i - 1] += x[ix] * -q0;
          ix++;
        }
      }

      iy += 7;
      jp1j += 7;
    }
  }

  for (i = 0; i < 7; i++) {
    p[i] = static_cast<signed char>(i + 1);
  }

  for (k = 0; k < 6; k++) {
    signed char i2;
    i2 = ipiv[k];
    if (i2 > k + 1) {
      iy = p[i2 - 1];
      p[i2 - 1] = p[k];
      p[k] = static_cast<signed char>(iy);
    }
  }

  for (k = 0; k < 7; k++) {
    b_tmp = 7 * (p[k] - 1);
    b[k + b_tmp] = 1.0;
    for (j = k + 1; j < 8; j++) {
      i = (j + b_tmp) - 1;
      if (b[i] != 0.0) {
        i1 = j + 1;
        for (b_i = i1; b_i < 8; b_i++) {
          a = (b_i + b_tmp) - 1;
          b[a] -= b[i] * x[(b_i + 7 * (j - 1)) - 1];
        }
      }
    }
  }

  for (j = 0; j < 7; j++) {
    jp1j = 7 * j;
    for (k = 6; k >= 0; k--) {
      iy = 7 * k;
      i = k + jp1j;
      q0 = b[i];
      if (q0 != 0.0) {
        b[i] = q0 / x[k + iy];
        for (b_i = 0; b_i < k; b_i++) {
          b_tmp = b_i + jp1j;
          b[b_tmp] -= b[i] * x[b_i + iy];
        }
      }
    }
  }

  for (i = 0; i < 16; i++) {
    for (i1 = 0; i1 < 7; i1++) {
      q0 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        q0 += this->P[i + (jp1j << 4)] * static_cast<double>(iv3[jp1j + (i1 << 4)]);
      }

      b_a[i + (i1 << 4)] = q0;
    }

    q0 = 0.0;
    for (i1 = 0; i1 < 7; i1++) {
      q1 = 0.0;
      for (jp1j = 0; jp1j < 7; jp1j++) {
        q1 += b_a[i + (jp1j << 4)] * b[jp1j + 7 * i1];
      }

      K[i + (i1 << 4)] = q1;
      q0 += q1 * this->r[i1];
    }

    this->x_hat[i] += q0;
  }

  std::memset(&c_EKF[0], 0, 256U * sizeof(double));
  for (k = 0; k < 16; k++) {
    c_EKF[k + (k << 4)] = 1.0;
  }

  for (i = 0; i < 16; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      q0 = 0.0;
      for (jp1j = 0; jp1j < 7; jp1j++) {
        q0 += K[i + (jp1j << 4)] * static_cast<double>(c_a[jp1j + 7 * i1]);
      }

      iy = i + (i1 << 4);
      Pdot2_tmp[iy] = c_EKF[iy] - q0;
    }

    for (i1 = 0; i1 < 16; i1++) {
      q0 = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        q0 += Pdot2_tmp[i + (jp1j << 4)] * this->P[jp1j + (i1 << 4)];
      }

      c_EKF[i + (i1 << 4)] = q0;
    }
  }

  std::memcpy(&this->P[0], &c_EKF[0], 256U * sizeof(double));
}

//
// Arguments    : const double x0[16]
// Return Type  : EKF *
//
EKF *EKF::init(const double x0[16])
{
  static const double dv[16] = { 0.040000000000000008, 0.040000000000000008,
    0.040000000000000008, 0.0016, 0.0016, 0.0016, 0.0036, 0.0036, 0.0036, 0.0036,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const double dv1[7] = { 1.0E-8, 1.0E-8, 1.0E-8, 0.0001, 0.0001, 0.0001,
    0.0001 };

  EKF *b_EKF;
  double b_I[256];
  double d[49];
  int i;
  b_EKF = this;

  // EKF -> extended kalman filter class for codegen
  //  public variables
  //  private variables
  // EKF Construct an instance of the EKF class
  std::memset(&b_I[0], 0, 256U * sizeof(double));
  for (i = 0; i < 16; i++) {
    b_I[i + (i << 4)] = 1.0;
  }

  for (i = 0; i < 256; i++) {
    b_EKF->P[i] = b_I[i];
  }

  std::memset(&b_I[0], 0, 256U * sizeof(double));
  for (i = 0; i < 16; i++) {
    b_I[i + (i << 4)] = dv[i];
  }

  for (i = 0; i < 256; i++) {
    b_EKF->Q[i] = b_I[i];
  }

  // pos_x  pos_y  pos_z  vel_x  vel_y  vel_z  q0    q1    q2    q3  %b_ax  b_ay  b_az  b_wx  b_wy  b_wz 
  std::memset(&d[0], 0, 49U * sizeof(double));
  for (i = 0; i < 7; i++) {
    d[i + 7 * i] = dv1[i];
  }

  for (i = 0; i < 49; i++) {
    b_EKF->R[i] = d[i];
  }

  // pos_x  pos_y  pos_z  q0    q1    q2    q3
  for (i = 0; i < 16; i++) {
    b_EKF->x_hat[i] = x0[i];
  }

  return b_EKF;
}

//
// File trailer for EKF.cpp
//
// [EOF]
//
