//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: expm.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 09-Apr-2022 14:02:55
//

// Include Files
#include "expm.h"
#include "rt_nonfinite.h"
#include "xzgetrf.h"
#include <algorithm>
#include <cmath>
#include <math.h>

// Function Declarations
namespace coder {
static void PadeApproximantOfDegree(const double A[36], unsigned char m,
                                    double F[36]);

}
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : const double A[36]
//                unsigned char m
//                double F[36]
// Return Type  : void
//
namespace coder {
static void PadeApproximantOfDegree(const double A[36], unsigned char m,
                                    double F[36])
{
  double A2[36];
  double A3[36];
  double A4[36];
  double V[36];
  double b_A4[36];
  double d;
  int ipiv[6];
  int F_tmp;
  int b_i;
  int i;
  int i1;
  int j;
  int jBcol;
  int k;
  int kAcol;
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (jBcol = 0; jBcol < 6; jBcol++) {
        d += A[i + 6 * jBcol] * A[jBcol + 6 * i1];
      }
      A2[i + 6 * i1] = d;
    }
  }
  if (m == 3) {
    std::copy(&A2[0], &A2[36], &F[0]);
    for (k = 0; k < 6; k++) {
      F_tmp = k + 6 * k;
      F[F_tmp] += 60.0;
    }
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          d += A[i + 6 * jBcol] * F[jBcol + 6 * i1];
        }
        A4[i + 6 * i1] = d;
      }
    }
    for (i = 0; i < 36; i++) {
      F[i] = A4[i];
      V[i] = 12.0 * A2[i];
    }
    d = 120.0;
  } else {
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          d += A2[i + 6 * jBcol] * A2[jBcol + 6 * i1];
        }
        A3[i + 6 * i1] = d;
      }
    }
    if (m == 5) {
      for (i = 0; i < 36; i++) {
        F[i] = A3[i] + 420.0 * A2[i];
      }
      for (k = 0; k < 6; k++) {
        F_tmp = k + 6 * k;
        F[F_tmp] += 15120.0;
      }
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          d = 0.0;
          for (jBcol = 0; jBcol < 6; jBcol++) {
            d += A[i + 6 * jBcol] * F[jBcol + 6 * i1];
          }
          A4[i + 6 * i1] = d;
        }
      }
      for (i = 0; i < 36; i++) {
        F[i] = A4[i];
        V[i] = 30.0 * A3[i] + 3360.0 * A2[i];
      }
      d = 30240.0;
    } else {
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          d = 0.0;
          for (jBcol = 0; jBcol < 6; jBcol++) {
            d += A3[i + 6 * jBcol] * A2[jBcol + 6 * i1];
          }
          b_A4[i + 6 * i1] = d;
        }
      }
      if (m == 7) {
        for (i = 0; i < 36; i++) {
          F[i] = (b_A4[i] + 1512.0 * A3[i]) + 277200.0 * A2[i];
        }
        for (k = 0; k < 6; k++) {
          F_tmp = k + 6 * k;
          F[F_tmp] += 8.64864E+6;
        }
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 6; i1++) {
            d = 0.0;
            for (jBcol = 0; jBcol < 6; jBcol++) {
              d += A[i + 6 * jBcol] * F[jBcol + 6 * i1];
            }
            A4[i + 6 * i1] = d;
          }
        }
        for (i = 0; i < 36; i++) {
          F[i] = A4[i];
          V[i] = (56.0 * b_A4[i] + 25200.0 * A3[i]) + 1.99584E+6 * A2[i];
        }
        d = 1.729728E+7;
      } else if (m == 9) {
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 6; i1++) {
            d = 0.0;
            for (jBcol = 0; jBcol < 6; jBcol++) {
              d += b_A4[i + 6 * jBcol] * A2[jBcol + 6 * i1];
            }
            V[i + 6 * i1] = d;
          }
        }
        for (i = 0; i < 36; i++) {
          F[i] = ((V[i] + 3960.0 * b_A4[i]) + 2.16216E+6 * A3[i]) +
                 3.027024E+8 * A2[i];
        }
        for (k = 0; k < 6; k++) {
          F_tmp = k + 6 * k;
          F[F_tmp] += 8.8216128E+9;
        }
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 6; i1++) {
            d = 0.0;
            for (jBcol = 0; jBcol < 6; jBcol++) {
              d += A[i + 6 * jBcol] * F[jBcol + 6 * i1];
            }
            A4[i + 6 * i1] = d;
          }
        }
        for (i = 0; i < 36; i++) {
          F[i] = A4[i];
          V[i] = ((90.0 * V[i] + 110880.0 * b_A4[i]) + 3.027024E+7 * A3[i]) +
                 2.0756736E+9 * A2[i];
        }
        d = 1.76432256E+10;
      } else {
        for (i = 0; i < 36; i++) {
          F[i] = (3.352212864E+10 * b_A4[i] + 1.05594705216E+13 * A3[i]) +
                 1.1873537964288E+15 * A2[i];
        }
        for (k = 0; k < 6; k++) {
          F_tmp = k + 6 * k;
          F[F_tmp] += 3.238237626624E+16;
        }
        for (i = 0; i < 36; i++) {
          A4[i] = (b_A4[i] + 16380.0 * A3[i]) + 4.08408E+7 * A2[i];
        }
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 6; i1++) {
            d = 0.0;
            for (jBcol = 0; jBcol < 6; jBcol++) {
              d += b_A4[i + 6 * jBcol] * A4[jBcol + 6 * i1];
            }
            jBcol = i + 6 * i1;
            V[jBcol] = d + F[jBcol];
          }
        }
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 6; i1++) {
            d = 0.0;
            for (jBcol = 0; jBcol < 6; jBcol++) {
              d += A[i + 6 * jBcol] * V[jBcol + 6 * i1];
            }
            F[i + 6 * i1] = d;
          }
        }
        for (i = 0; i < 36; i++) {
          A4[i] = (182.0 * b_A4[i] + 960960.0 * A3[i]) + 1.32324192E+9 * A2[i];
        }
        for (i = 0; i < 6; i++) {
          for (i1 = 0; i1 < 6; i1++) {
            d = 0.0;
            for (jBcol = 0; jBcol < 6; jBcol++) {
              d += b_A4[i + 6 * jBcol] * A4[jBcol + 6 * i1];
            }
            jBcol = i + 6 * i1;
            V[jBcol] = ((d + 6.704425728E+11 * b_A4[jBcol]) +
                        1.29060195264E+14 * A3[jBcol]) +
                       7.7717703038976E+15 * A2[jBcol];
          }
        }
        d = 6.476475253248E+16;
      }
    }
  }
  for (k = 0; k < 6; k++) {
    jBcol = k + 6 * k;
    V[jBcol] += d;
  }
  for (k = 0; k < 36; k++) {
    d = F[k];
    V[k] -= d;
    d *= 2.0;
    F[k] = d;
  }
  internal::reflapack::xzgetrf(V, ipiv, &jBcol);
  for (b_i = 0; b_i < 5; b_i++) {
    i = ipiv[b_i];
    if (i != b_i + 1) {
      for (j = 0; j < 6; j++) {
        jBcol = b_i + 6 * j;
        d = F[jBcol];
        F_tmp = (i + 6 * j) - 1;
        F[jBcol] = F[F_tmp];
        F[F_tmp] = d;
      }
    }
  }
  for (j = 0; j < 6; j++) {
    jBcol = 6 * j;
    for (k = 0; k < 6; k++) {
      kAcol = 6 * k;
      i = k + jBcol;
      if (F[i] != 0.0) {
        i1 = k + 2;
        for (b_i = i1; b_i < 7; b_i++) {
          F_tmp = (b_i + jBcol) - 1;
          F[F_tmp] -= F[i] * V[(b_i + kAcol) - 1];
        }
      }
    }
  }
  for (j = 0; j < 6; j++) {
    jBcol = 6 * j;
    for (k = 5; k >= 0; k--) {
      kAcol = 6 * k;
      i = k + jBcol;
      d = F[i];
      if (d != 0.0) {
        F[i] = d / V[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          F_tmp = b_i + jBcol;
          F[F_tmp] -= F[i] * V[b_i + kAcol];
        }
      }
    }
  }
  for (k = 0; k < 6; k++) {
    F_tmp = k + 6 * k;
    F[F_tmp]++;
  }
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
} // namespace coder
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : double A[36]
//                double F[36]
// Return Type  : void
//
namespace coder {
void expm(double A[36], double F[36])
{
  static const double theta[5]{0.01495585217958292, 0.253939833006323,
                               0.95041789961629319, 2.097847961257068,
                               5.3719203511481517};
  static const unsigned char uv[5]{3U, 5U, 7U, 9U, 13U};
  double b_F[36];
  double normA;
  double s;
  int eint;
  int i;
  int j;
  boolean_T exitg1;
  normA = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 6)) {
    s = 0.0;
    for (i = 0; i < 6; i++) {
      s += std::abs(A[i + 6 * j]);
    }
    if (std::isnan(s)) {
      normA = rtNaN;
      exitg1 = true;
    } else {
      if (s > normA) {
        normA = s;
      }
      j++;
    }
  }
  if (normA <= 5.3719203511481517) {
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 5)) {
      if (normA <= theta[i]) {
        PadeApproximantOfDegree(A, uv[i], F);
        exitg1 = true;
      } else {
        i++;
      }
    }
  } else {
    normA /= 5.3719203511481517;
    if ((!std::isinf(normA)) && (!std::isnan(normA))) {
      normA = frexp(normA, &eint);
    } else {
      eint = 0;
    }
    s = eint;
    if (normA == 0.5) {
      s = static_cast<double>(eint) - 1.0;
    }
    normA = rt_powd_snf(2.0, s);
    for (i = 0; i < 36; i++) {
      A[i] /= normA;
    }
    PadeApproximantOfDegree(A, 13U, F);
    i = static_cast<int>(s);
    for (j = 0; j < i; j++) {
      for (eint = 0; eint < 6; eint++) {
        for (int b_i{0}; b_i < 6; b_i++) {
          normA = 0.0;
          for (int i1{0}; i1 < 6; i1++) {
            normA += F[eint + 6 * i1] * F[i1 + 6 * b_i];
          }
          b_F[eint + 6 * b_i] = normA;
        }
      }
      std::copy(&b_F[0], &b_F[36], &F[0]);
    }
  }
}

} // namespace coder

//
// File trailer for expm.cpp
//
// [EOF]
//
