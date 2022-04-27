//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 27-Apr-2022 11:15:15
//

// Include Files
#include "inv.h"
#include "rt_nonfinite.h"
#include "xzgetrf.h"

// Function Definitions
//
// Arguments    : const double x[36]
//                double y[36]
// Return Type  : void
//
namespace coder
{
  void inv(const double x[36], double y[36])
  {
    double b_x[36];
    int ipiv[6];
    int b_i;
    int i;
    int j;
    int k;
    int kAcol;
    int pipk;
    int y_tmp;
    signed char p[6];
    for (i = 0; i < 36; i++) {
      y[i] = 0.0;
      b_x[i] = x[i];
    }

    internal::reflapack::xzgetrf(b_x, ipiv, &pipk);
    for (i = 0; i < 6; i++) {
      p[i] = static_cast<signed char>(i + 1);
    }

    for (k = 0; k < 5; k++) {
      i = ipiv[k];
      if (i > k + 1) {
        pipk = p[i - 1];
        p[i - 1] = p[k];
        p[k] = static_cast<signed char>(pipk);
      }
    }

    for (k = 0; k < 6; k++) {
      y_tmp = 6 * (p[k] - 1);
      y[k + y_tmp] = 1.0;
      for (j = k + 1; j < 7; j++) {
        i = (j + y_tmp) - 1;
        if (y[i] != 0.0) {
          pipk = j + 1;
          for (b_i = pipk; b_i < 7; b_i++) {
            kAcol = (b_i + y_tmp) - 1;
            y[kAcol] -= y[i] * b_x[(b_i + 6 * (j - 1)) - 1];
          }
        }
      }
    }

    for (j = 0; j < 6; j++) {
      pipk = 6 * j;
      for (k = 5; k >= 0; k--) {
        double d;
        kAcol = 6 * k;
        i = k + pipk;
        d = y[i];
        if (d != 0.0) {
          y[i] = d / b_x[k + kAcol];
          for (b_i = 0; b_i < k; b_i++) {
            y_tmp = b_i + pipk;
            y[y_tmp] -= y[i] * b_x[b_i + kAcol];
          }
        }
      }
    }
  }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
