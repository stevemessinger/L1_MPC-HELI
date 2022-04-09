//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_foo_api.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 09-Apr-2022 14:02:55
//

#ifndef _CODER_FOO_API_H
#define _CODER_FOO_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void foo(real_T x[13], real_T u_mpc[4], real_T dt, real_T u[4]);

void foo_api(const mxArray *const prhs[3], const mxArray **plhs);

void foo_atexit();

void foo_initialize();

void foo_terminate();

void foo_xil_shutdown();

void foo_xil_terminate();

#endif
//
// File trailer for _coder_foo_api.h
//
// [EOF]
//
