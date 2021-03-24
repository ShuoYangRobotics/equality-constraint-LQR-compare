//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xnrm2.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//
#ifndef XNRM2_H
#define XNRM2_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      double b_xnrm2(int n, const ::coder::array<double, 2U> &x, int ix0);
      double xnrm2(int n, const double x_data[], int ix0);
      double xnrm2(int n, const ::coder::array<double, 2U> &x, int ix0);
      double xnrm2(const double x[3], int ix0);
      double xnrm2(int n, const ::coder::array<double, 1U> &x, int ix0);
    }
  }
}

#endif

//
// File trailer for xnrm2.h
//
// [EOF]
//
