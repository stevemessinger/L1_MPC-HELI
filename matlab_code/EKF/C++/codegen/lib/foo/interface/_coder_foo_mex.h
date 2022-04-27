/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_foo_mex.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 27-Apr-2022 13:20:00
 */

#ifndef _CODER_FOO_MEX_H
#define _CODER_FOO_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void foo_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs, const
                       mxArray *prhs[4]);
  MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T
    nrhs, const mxArray *prhs[]);
  emlrtCTX mexFunctionCreateRootTLS(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_foo_mex.h
 *
 * [EOF]
 */
