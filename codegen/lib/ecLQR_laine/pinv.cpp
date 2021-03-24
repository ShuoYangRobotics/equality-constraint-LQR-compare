//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pinv.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "pinv.h"
#include "rt_nonfinite.h"
#include "svd1.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>
#include <math.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                ::coder::array<double, 2U> &X
// Return Type  : void
//
namespace coder
{
  void b_pinv(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &X)
  {
    array<double, 2U> U;
    array<double, 2U> x;
    double b_V_data[16];
    double V_data[9];
    double s_data[3];
    double absx;
    int V_size[2];
    int s_size[1];
    int ar;
    int i;
    int ib;
    int k;
    int m;
    int n;
    int nx;
    int vcol;
    m = A.size(0);
    n = A.size(1);
    X.set_size(A.size(1), A.size(0));
    nx = A.size(1) * A.size(0);
    for (i = 0; i < nx; i++) {
      X[i] = 0.0;
    }

    if (A.size(1) != 0) {
      boolean_T p;
      nx = A.size(0) * A.size(1);
      p = true;
      for (k = 0; k < nx; k++) {
        if ((!p) || (rtIsInf(A[k]) || rtIsNaN(A[k]))) {
          p = false;
        }
      }

      if (!p) {
        X.set_size(A.size(1), A.size(0));
        nx = A.size(1) * A.size(0);
        for (i = 0; i < nx; i++) {
          X[i] = rtNaN;
        }
      } else {
        int ia;
        int r;
        internal::c_svd(A, U, s_data, s_size, V_data, V_size);
        ar = V_size[0];
        ia = V_size[1];
        nx = V_size[0] * V_size[1];
        if (0 <= nx - 1) {
          std::memcpy(&b_V_data[0], &V_data[0], nx * sizeof(double));
        }

        absx = std::abs(s_data[0]);
        if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
          if (absx <= 2.2250738585072014E-308) {
            absx = 4.94065645841247E-324;
          } else {
            frexp(absx, &vcol);
            absx = std::ldexp(1.0, vcol - 53);
          }
        } else {
          absx = rtNaN;
        }

        absx *= static_cast<double>(A.size(0));
        r = -1;
        k = 0;
        while ((k <= n - 1) && (s_data[k] > absx)) {
          r++;
          k++;
        }

        if (r + 1 > 0) {
          int i1;
          int ic;
          int j;
          vcol = 1;
          for (j = 0; j <= r; j++) {
            absx = 1.0 / s_data[j];
            x.set_size(ar, ia);
            nx = ar * ia;
            for (i = 0; i < nx; i++) {
              x[i] = b_V_data[i];
            }

            i = vcol + n;
            i1 = i - 1;
            for (k = vcol; k <= i1; k++) {
              x[k - 1] = absx * x[k - 1];
            }

            ar = x.size(0);
            ia = x.size(1);
            nx = x.size(0) * x.size(1);
            for (i1 = 0; i1 < nx; i1++) {
              b_V_data[i1] = x[i1];
            }

            vcol = i;
          }

          nx = A.size(1) * (A.size(0) - 1);
          for (k = 0; n < 0 ? k >= nx : k <= nx; k += n) {
            i = k + 1;
            i1 = k + n;
            for (ic = i; ic <= i1; ic++) {
              X[ic - 1] = 0.0;
            }
          }

          vcol = 0;
          for (k = 0; n < 0 ? k >= nx : k <= nx; k += n) {
            ar = -1;
            vcol++;
            i = vcol + m * r;
            for (ib = vcol; m < 0 ? ib >= i : ib <= i; ib += m) {
              ia = ar;
              i1 = k + 1;
              j = k + n;
              for (ic = i1; ic <= j; ic++) {
                ia++;
                X[ic - 1] = X[ic - 1] + U[ib - 1] * b_V_data[ia];
              }

              ar += n;
            }
          }
        }
      }
    }
  }

  //
  // Arguments    : const ::coder::array<double, 2U> &A
  //                ::coder::array<double, 2U> &X
  // Return Type  : void
  //
  void pinv(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &X)
  {
    array<double, 2U> U;
    double V_data[9];
    double s_data[3];
    double absx;
    int V_size[2];
    int s_size[1];
    int ar;
    int br;
    int cr;
    int exponent;
    int i;
    int ib;
    int m;
    int nx;
    boolean_T p;
    m = A.size(0);
    X.set_size(3, A.size(0));
    nx = 3 * A.size(0);
    for (i = 0; i < nx; i++) {
      X[i] = 0.0;
    }

    nx = A.size(0) * 3;
    p = true;
    for (br = 0; br < nx; br++) {
      if ((!p) || (rtIsInf(A[br]) || rtIsNaN(A[br]))) {
        p = false;
      }
    }

    if (!p) {
      X.set_size(3, A.size(0));
      nx = 3 * A.size(0);
      for (i = 0; i < nx; i++) {
        X[i] = rtNaN;
      }
    } else {
      int r;
      internal::b_svd(A, U, s_data, s_size, V_data, V_size);
      absx = std::abs(s_data[0]);
      if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
        if (absx <= 2.2250738585072014E-308) {
          absx = 4.94065645841247E-324;
        } else {
          frexp(absx, &exponent);
          absx = std::ldexp(1.0, exponent - 53);
        }
      } else {
        absx = rtNaN;
      }

      absx *= static_cast<double>(A.size(0));
      r = -1;
      br = 0;
      while ((br < 3) && (s_data[br] > absx)) {
        r++;
        br++;
      }

      if (r + 1 > 0) {
        int ic;
        nx = 1;
        for (exponent = 0; exponent <= r; exponent++) {
          absx = 1.0 / s_data[exponent];
          i = nx + 2;
          for (br = nx; br <= i; br++) {
            V_data[br - 1] *= absx;
          }

          nx += 3;
        }

        nx = 3 * (A.size(0) - 1);
        for (cr = 0; cr <= nx; cr += 3) {
          i = cr + 1;
          exponent = cr + 3;
          for (ic = i; ic <= exponent; ic++) {
            X[ic - 1] = 0.0;
          }
        }

        br = 0;
        for (cr = 0; cr <= nx; cr += 3) {
          ar = -1;
          br++;
          i = br + m * r;
          for (ib = br; m < 0 ? ib >= i : ib <= i; ib += m) {
            int i1;
            int ia;
            ia = ar;
            exponent = cr + 1;
            i1 = cr + 3;
            for (ic = exponent; ic <= i1; ic++) {
              ia++;
              X[ic - 1] = X[ic - 1] + U[ib - 1] * V_data[ia];
            }

            ar += 3;
          }
        }
      }
    }
  }
}

//
// File trailer for pinv.cpp
//
// [EOF]
//
