//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xswap.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "xswap.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : double x[9]
//                int ix0
//                int iy0
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      void xswap(double x[9], int ix0, int iy0)
      {
        double temp;
        temp = x[ix0 - 1];
        x[ix0 - 1] = x[iy0 - 1];
        x[iy0 - 1] = temp;
        temp = x[ix0];
        x[ix0] = x[iy0];
        x[iy0] = temp;
        temp = x[ix0 + 1];
        x[ix0 + 1] = x[iy0 + 1];
        x[iy0 + 1] = temp;
      }
    }
  }
}

//
// File trailer for xswap.cpp
//
// [EOF]
//
