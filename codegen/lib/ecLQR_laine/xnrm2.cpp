//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xnrm2.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : int n
//                const ::coder::array<double, 2U> &x
//                int ix0
// Return Type  : double
//
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      double b_xnrm2(int n, const ::coder::array<double, 2U> &x, int ix0)
      {
        double y;
        y = 0.0;
        if (n >= 1) {
          if (n == 1) {
            y = std::abs(x[ix0 - 1]);
          } else {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = (ix0 + n) - 1;
            for (int k = ix0; k <= kend; k++) {
              double absxk;
              absxk = std::abs(x[k - 1]);
              if (absxk > scale) {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              } else {
                double t;
                t = absxk / scale;
                y += t * t;
              }
            }

            y = scale * std::sqrt(y);
          }
        }

        return y;
      }

      //
      // Arguments    : int n
      //                const double x_data[]
      //                int ix0
      // Return Type  : double
      //
      double xnrm2(int n, const double x_data[], int ix0)
      {
        double y;
        y = 0.0;
        if (n >= 1) {
          if (n == 1) {
            y = std::abs(x_data[ix0 - 1]);
          } else {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = (ix0 + n) - 1;
            for (int k = ix0; k <= kend; k++) {
              double absxk;
              absxk = std::abs(x_data[k - 1]);
              if (absxk > scale) {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              } else {
                double t;
                t = absxk / scale;
                y += t * t;
              }
            }

            y = scale * std::sqrt(y);
          }
        }

        return y;
      }

      //
      // Arguments    : int n
      //                const ::coder::array<double, 2U> &x
      //                int ix0
      // Return Type  : double
      //
      double xnrm2(int n, const ::coder::array<double, 2U> &x, int ix0)
      {
        double y;
        y = 0.0;
        if (n == 1) {
          y = std::abs(x[ix0 - 1]);
        } else {
          double scale;
          int kend;
          scale = 3.3121686421112381E-170;
          kend = (ix0 + n) - 1;
          for (int k = ix0; k <= kend; k++) {
            double absxk;
            absxk = std::abs(x[k - 1]);
            if (absxk > scale) {
              double t;
              t = scale / absxk;
              y = y * t * t + 1.0;
              scale = absxk;
            } else {
              double t;
              t = absxk / scale;
              y += t * t;
            }
          }

          y = scale * std::sqrt(y);
        }

        return y;
      }

      //
      // Arguments    : const double x[3]
      //                int ix0
      // Return Type  : double
      //
      double xnrm2(const double x[3], int ix0)
      {
        double scale;
        double y;
        int kend;
        y = 0.0;
        scale = 3.3121686421112381E-170;
        kend = ix0 + 1;
        for (int k = ix0; k <= kend; k++) {
          double absxk;
          absxk = std::abs(x[k - 1]);
          if (absxk > scale) {
            double t;
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
          } else {
            double t;
            t = absxk / scale;
            y += t * t;
          }
        }

        return scale * std::sqrt(y);
      }

      //
      // Arguments    : int n
      //                const ::coder::array<double, 1U> &x
      //                int ix0
      // Return Type  : double
      //
      double xnrm2(int n, const ::coder::array<double, 1U> &x, int ix0)
      {
        double y;
        y = 0.0;
        if (n >= 1) {
          if (n == 1) {
            y = std::abs(x[ix0 - 1]);
          } else {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = (ix0 + n) - 1;
            for (int k = ix0; k <= kend; k++) {
              double absxk;
              absxk = std::abs(x[k - 1]);
              if (absxk > scale) {
                double t;
                t = scale / absxk;
                y = y * t * t + 1.0;
                scale = absxk;
              } else {
                double t;
                t = absxk / scale;
                y += t * t;
              }
            }

            y = scale * std::sqrt(y);
          }
        }

        return y;
      }
    }
  }
}

//
// File trailer for xnrm2.cpp
//
// [EOF]
//
