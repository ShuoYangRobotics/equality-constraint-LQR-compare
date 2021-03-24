//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//
#ifndef SVD1_H
#define SVD1_H

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
    void b_svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U>
               &U, double s_data[], int s_size[1], double V[9]);
    void b_svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U>
               &U, double s_data[], int s_size[1], double V_data[], int V_size[2]);
    void b_svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U>
               &U, ::coder::array<double, 1U> &s, ::coder::array<double, 2U> &V);
    void c_svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U>
               &U, double s_data[], int s_size[1], double V_data[], int V_size[2]);
  }
}

#endif

//
// File trailer for svd1.h
//
// [EOF]
//
