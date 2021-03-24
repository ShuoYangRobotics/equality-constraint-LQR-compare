//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdotc.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "xdotc.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const double x[9]
//                const double y[9]
//                int iy0
// Return Type  : double
//
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      double xdotc(const double x[9], const double y[9], int iy0)
      {
        return x[1] * y[iy0 - 1] + x[2] * y[iy0];
      }
    }
  }
}

//
// File trailer for xdotc.cpp
//
// [EOF]
//
