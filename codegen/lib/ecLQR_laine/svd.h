//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//
#ifndef SVD_H
#define SVD_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  void svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &U, ::
           coder::array<double, 2U> &S, double V[9]);
  void svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &U, ::
           coder::array<double, 2U> &S, ::coder::array<double, 2U> &V);
}

#endif

//
// File trailer for svd.h
//
// [EOF]
//
