//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ecLQR_laine_types.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//
#ifndef ECLQR_LAINE_TYPES_H
#define ECLQR_LAINE_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
struct struct0_T
{
  double nx;
  double nu;
  double ncxu;
  double ncx;
  double x0[3];
  double xN[3];
  double LQR_time;
  double dt;
  double N;
  double A[9];
  double B[9];
  double Q[9];
  double R[9];
  double Qf[9];
  double simulation_noise;
  double Cxu;
};

#endif

//
// File trailer for ecLQR_laine_types.h
//
// [EOF]
//
