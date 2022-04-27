/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_foo_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 27-Apr-2022 11:15:15
 */

#ifndef _CODER_FOO_API_H
#define _CODER_FOO_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void foo(real_T x[13], real_T u_mpc[4], real_T dt, real_T adaptiveGain, real_T
           cutoffFrequency, real_T u[4], real_T a[6], real_T *b, real_T u2[4],
           real_T a2[6], real_T *b2);
  void foo_api(const mxArray * const prhs[5], int32_T nlhs, const mxArray *plhs
               [6]);
  void foo_atexit(void);
  void foo_initialize(void);
  void foo_terminate(void);
  void foo_xil_shutdown(void);
  void foo_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_foo_api.h
 *
 * [EOF]
 */
