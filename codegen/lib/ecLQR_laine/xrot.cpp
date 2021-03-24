//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "xrot.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : double x[9]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      void xrot(double x[9], int ix0, int iy0, double c, double s)
      {
        double temp;
        double temp_tmp;
        temp = x[iy0 - 1];
        temp_tmp = x[ix0 - 1];
        x[iy0 - 1] = c * temp - s * temp_tmp;
        x[ix0 - 1] = c * temp_tmp + s * temp;
        temp = c * x[ix0] + s * x[iy0];
        x[iy0] = c * x[iy0] - s * x[ix0];
        x[ix0] = temp;
        temp = x[iy0 + 1];
        temp_tmp = x[ix0 + 1];
        x[iy0 + 1] = c * temp - s * temp_tmp;
        x[ix0 + 1] = c * temp_tmp + s * temp;
      }
    }
  }
}

//
// File trailer for xrot.cpp
//
// [EOF]
//
