//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pinv.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//
#ifndef PINV_H
#define PINV_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  void b_pinv(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &X);
  void pinv(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &X);
}

#endif

//
// File trailer for pinv.h
//
// [EOF]
//
