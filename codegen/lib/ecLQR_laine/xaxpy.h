//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//
#ifndef XAXPY_H
#define XAXPY_H

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
      void b_xaxpy(int n, double a, int ix0, ::coder::array<double, 2U> &y, int
                   iy0);
      void xaxpy(int n, double a, int ix0, ::coder::array<double, 2U> &y, int
                 iy0);
      void xaxpy(double a, double y[9], int iy0);
      void xaxpy(int n, double a, const ::coder::array<double, 2U> &x, int ix0, ::
                 coder::array<double, 1U> &y, int iy0);
      void xaxpy(int n, double a, const ::coder::array<double, 1U> &x, int ix0, ::
                 coder::array<double, 2U> &y, int iy0);
    }
  }
}

#endif

//
// File trailer for xaxpy.h
//
// [EOF]
//
