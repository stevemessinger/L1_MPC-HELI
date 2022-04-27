//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzgetrf.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 27-Apr-2022 11:15:15
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
namespace coder
{
  namespace internal
  {
    namespace reflapack
    {
      void xzgetrf(double A[36], int ipiv[6], int *info)
      {
        int i;
        int ix;
        int iy;
        int jA;
        for (i = 0; i < 6; i++) {
          ipiv[i] = i + 1;
        }

        *info = 0;
        for (int j = 0; j < 5; j++) {
          double smax;
          int b_tmp;
          int jp1j;
          int k;
          int mmj_tmp;
          mmj_tmp = 4 - j;
          b_tmp = j * 7;
          jp1j = b_tmp + 2;
          iy = 6 - j;
          jA = 0;
          ix = b_tmp;
          smax = std::abs(A[b_tmp]);
          for (k = 2; k <= iy; k++) {
            double s;
            ix++;
            s = std::abs(A[ix]);
            if (s > smax) {
              jA = k - 1;
              smax = s;
            }
          }

          if (A[b_tmp + jA] != 0.0) {
            if (jA != 0) {
              iy = j + jA;
              ipiv[j] = iy + 1;
              ix = j;
              for (k = 0; k < 6; k++) {
                smax = A[ix];
                A[ix] = A[iy];
                A[iy] = smax;
                ix += 6;
                iy += 6;
              }
            }

            i = (b_tmp - j) + 6;
            for (iy = jp1j; iy <= i; iy++) {
              A[iy - 1] /= A[b_tmp];
            }
          } else {
            *info = j + 1;
          }

          iy = b_tmp + 6;
          jA = b_tmp;
          for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
            smax = A[iy];
            if (A[iy] != 0.0) {
              ix = b_tmp + 1;
              i = jA + 8;
              k = (jA - j) + 12;
              for (int ijA = i; ijA <= k; ijA++) {
                A[ijA - 1] += A[ix] * -smax;
                ix++;
              }
            }

            iy += 6;
            jA += 6;
          }
        }

        if ((*info == 0) && (!(A[35] != 0.0))) {
          *info = 6;
        }
      }
    }
  }
}

//
// File trailer for xzgetrf.cpp
//
// [EOF]
//
