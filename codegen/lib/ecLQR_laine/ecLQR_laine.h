//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ecLQR_laine.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//
#ifndef ECLQR_LAINE_H
#define ECLQR_LAINE_H

// Include Files
#include "ecLQR_laine_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void ecLQR_laine(const struct0_T *param, const double xN[3], const double
  A_list[900], const double B_list[900], const double C_list[900], const double
  D_list[900], const double G_list[900], const double r_list[300], const double
  h_list[300], coder::array<double, 2U> &x_list, coder::array<double, 3U>
  &K_list, coder::array<double, 2U> &k_list);

#endif

//
// File trailer for ecLQR_laine.h
//
// [EOF]
//
