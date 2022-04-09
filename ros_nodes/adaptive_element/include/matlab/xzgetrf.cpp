//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzgetrf.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 09-Apr-2022 14:54:34
//

// Include Files
#include "xzgetrf.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double A[36]
//                int ipiv[6]
//                int *info
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xzgetrf(double A[36], int ipiv[6], int *info)
{
  int i;
  for (i = 0; i < 6; i++) {
    ipiv[i] = i + 1;
  }
  *info = 0;
  for (int j{0}; j < 5; j++) {
    double smax;
    int a;
    int b_tmp;
    int jA;
    int jp1j;
    int k;
    int mmj_tmp;
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    jA = 6 - j;
    a = 0;
    smax = std::abs(A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = std::abs(A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (A[b_tmp + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = jA + 1;
        for (k = 0; k < 6; k++) {
          a = j + k * 6;
          smax = A[a];
          i = jA + k * 6;
          A[a] = A[i];
          A[i] = smax;
        }
      }
      i = (b_tmp - j) + 6;
      for (jA = jp1j; jA <= i; jA++) {
        A[jA - 1] /= A[b_tmp];
      }
    } else {
      *info = j + 1;
    }
    jA = b_tmp;
    for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
      smax = A[(b_tmp + jp1j * 6) + 6];
      if (smax != 0.0) {
        i = jA + 8;
        a = (jA - j) + 12;
        for (k = i; k <= a; k++) {
          A[k - 1] += A[((b_tmp + k) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
  }
  if ((*info == 0) && (!(A[35] != 0.0))) {
    *info = 6;
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzgetrf.cpp
//
// [EOF]
//
