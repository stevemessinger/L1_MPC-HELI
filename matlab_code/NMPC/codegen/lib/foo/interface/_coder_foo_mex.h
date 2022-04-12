//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_foo_mex.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 09-Apr-2022 14:54:34
//

#ifndef _CODER_FOO_MEX_H
#define _CODER_FOO_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_foo_mexFunction(int32_T nlhs, mxArray *plhs[6], int32_T nrhs,
                            const mxArray *prhs[5]);

#endif
//
// File trailer for _coder_foo_mex.h
//
// [EOF]
//
