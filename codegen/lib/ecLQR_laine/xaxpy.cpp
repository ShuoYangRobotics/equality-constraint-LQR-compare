//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "xaxpy.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : int n
//                double a
//                int ix0
//                ::coder::array<double, 2U> &y
//                int iy0
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      void b_xaxpy(int n, double a, int ix0, ::coder::array<double, 2U> &y, int
                   iy0)
      {
        if ((n >= 1) && (!(a == 0.0))) {
          int i;
          int ix;
          int iy;
          ix = ix0 - 1;
          iy = iy0 - 1;
          i = n - 1;
          for (int k = 0; k <= i; k++) {
            y[iy] = y[iy] + a * y[ix];
            ix++;
            iy++;
          }
        }
      }

      //
      // Arguments    : int n
      //                double a
      //                int ix0
      //                ::coder::array<double, 2U> &y
      //                int iy0
      // Return Type  : void
      //
      void xaxpy(int n, double a, int ix0, ::coder::array<double, 2U> &y, int
                 iy0)
      {
        if (!(a == 0.0)) {
          int i;
          int ix;
          int iy;
          ix = ix0 - 1;
          iy = iy0 - 1;
          i = n - 1;
          for (int k = 0; k <= i; k++) {
            y[iy] = y[iy] + a * y[ix];
            ix++;
            iy++;
          }
        }
      }

      //
      // Arguments    : double a
      //                double y[9]
      //                int iy0
      // Return Type  : void
      //
      void xaxpy(double a, double y[9], int iy0)
      {
        if (!(a == 0.0)) {
          y[iy0 - 1] += a * y[1];
          y[iy0] += a * y[2];
        }
      }

      //
      // Arguments    : int n
      //                double a
      //                const ::coder::array<double, 2U> &x
      //                int ix0
      //                ::coder::array<double, 1U> &y
      //                int iy0
      // Return Type  : void
      //
      void xaxpy(int n, double a, const ::coder::array<double, 2U> &x, int ix0, ::
                 coder::array<double, 1U> &y, int iy0)
      {
        if ((n >= 1) && (!(a == 0.0))) {
          int i;
          int ix;
          int iy;
          ix = ix0 - 1;
          iy = iy0 - 1;
          i = n - 1;
          for (int k = 0; k <= i; k++) {
            y[iy] = y[iy] + a * x[ix];
            ix++;
            iy++;
          }
        }
      }

      //
      // Arguments    : int n
      //                double a
      //                const ::coder::array<double, 1U> &x
      //                int ix0
      //                ::coder::array<double, 2U> &y
      //                int iy0
      // Return Type  : void
      //
      void xaxpy(int n, double a, const ::coder::array<double, 1U> &x, int ix0, ::
                 coder::array<double, 2U> &y, int iy0)
      {
        if ((n >= 1) && (!(a == 0.0))) {
          int i;
          int ix;
          int iy;
          ix = ix0 - 1;
          iy = iy0 - 1;
          i = n - 1;
          for (int k = 0; k <= i; k++) {
            y[iy] = y[iy] + a * x[ix];
            ix++;
            iy++;
          }
        }
      }
    }
  }
}

//
// File trailer for xaxpy.cpp
//
// [EOF]
//
