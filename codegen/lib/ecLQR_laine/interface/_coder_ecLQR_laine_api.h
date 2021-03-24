/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ecLQR_laine_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 24-Mar-2021 09:42:45
 */

#ifndef _CODER_ECLQR_LAINE_API_H
#define _CODER_ECLQR_LAINE_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  real_T nx;
  real_T nu;
  real_T ncxu;
  real_T ncx;
  real_T x0[3];
  real_T xN[3];
  real_T LQR_time;
  real_T dt;
  real_T N;
  real_T A[9];
  real_T B[9];
  real_T Q[9];
  real_T R[9];
  real_T Qf[9];
  real_T simulation_noise;
  real_T Cxu;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void ecLQR_laine(struct0_T *param, real_T xN[3], real_T A_list[900], real_T
                   B_list[900], real_T C_list[900], real_T D_list[900], real_T
                   G_list[900], real_T r_list[300], real_T h_list[300],
                   emxArray_real_T *x_list, emxArray_real_T *K_list,
                   emxArray_real_T *k_list);
  void ecLQR_laine_api(const mxArray * const prhs[9], int32_T nlhs, const
                       mxArray *plhs[3]);
  void ecLQR_laine_atexit(void);
  void ecLQR_laine_initialize(void);
  void ecLQR_laine_terminate(void);
  void ecLQR_laine_xil_shutdown(void);
  void ecLQR_laine_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_ecLQR_laine_api.h
 *
 * [EOF]
 */
