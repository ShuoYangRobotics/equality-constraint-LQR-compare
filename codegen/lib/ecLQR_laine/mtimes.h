//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//
#ifndef MTIMES_H
#define MTIMES_H

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
      void mtimes(const ::coder::array<double, 2U> &A, const double B[9], ::
                  coder::array<double, 2U> &C);
      void mtimes(const ::coder::array<double, 2U> &A, const double B_data[],
                  const int B_size[2], ::coder::array<double, 2U> &C);
      void mtimes(const double A_data[], const int A_size[2], const ::coder::
                  array<double, 2U> &B, ::coder::array<double, 2U> &C);
      void mtimes(const ::coder::array<double, 2U> &A, const ::coder::array<
                  double, 2U> &B, double C[9]);
      void mtimes(const double A[9], const ::coder::array<double, 2U> &B, ::
                  coder::array<double, 2U> &C);
      void mtimes(const ::coder::array<double, 2U> &A, const ::coder::array<
                  double, 2U> &B, ::coder::array<double, 2U> &C);
    }
  }
}

#endif

//
// File trailer for mtimes.h
//
// [EOF]
//
