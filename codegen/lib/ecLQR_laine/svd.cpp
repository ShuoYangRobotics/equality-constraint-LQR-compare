//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "svd.h"
#include "rt_nonfinite.h"
#include "svd1.h"
#include "coder_array.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                ::coder::array<double, 2U> &U
//                ::coder::array<double, 2U> &S
//                double V[9]
// Return Type  : void
//
namespace coder
{
  void svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &U, ::
           coder::array<double, 2U> &S, double V[9])
  {
    array<double, 2U> U1;
    array<double, 2U> r;
    double V1[9];
    double s_data[3];
    int s_size[1];
    int k;
    int nx;
    boolean_T p;
    nx = A.size(0) * 3;
    p = true;
    for (k = 0; k < nx; k++) {
      if ((!p) || (rtIsInf(A[k]) || rtIsNaN(A[k]))) {
        p = false;
      }
    }

    if (p) {
      internal::b_svd(A, U, s_data, s_size, V);
    } else {
      r.set_size(A.size(0), 3);
      nx = A.size(0) * 3;
      for (k = 0; k < nx; k++) {
        r[k] = 0.0;
      }

      internal::b_svd(r, U1, s_data, s_size, V1);
      U.set_size(U1.size(0), U1.size(1));
      nx = U1.size(0) * U1.size(1);
      for (k = 0; k < nx; k++) {
        U[k] = rtNaN;
      }

      s_data[0] = rtNaN;
      s_data[1] = rtNaN;
      s_data[2] = rtNaN;
      for (k = 0; k < 9; k++) {
        V[k] = rtNaN;
      }
    }

    S.set_size(U.size(1), 3);
    nx = U.size(1) * 3;
    for (k = 0; k < nx; k++) {
      S[k] = 0.0;
    }

    S[0] = s_data[0];
    S[S.size(0) + 1] = s_data[1];
    S[S.size(0) * 2 + 2] = s_data[2];
  }

  //
  // Arguments    : const ::coder::array<double, 2U> &A
  //                ::coder::array<double, 2U> &U
  //                ::coder::array<double, 2U> &S
  //                ::coder::array<double, 2U> &V
  // Return Type  : void
  //
  void svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U> &U, ::
           coder::array<double, 2U> &S, ::coder::array<double, 2U> &V)
  {
    array<double, 2U> U1;
    array<double, 2U> V1;
    array<double, 2U> r;
    array<double, 1U> s;
    int i;
    int k;
    int nx;
    boolean_T p;
    nx = A.size(0) * A.size(1);
    p = true;
    for (k = 0; k < nx; k++) {
      if ((!p) || (rtIsInf(A[k]) || rtIsNaN(A[k]))) {
        p = false;
      }
    }

    if (p) {
      internal::b_svd(A, U, s, V);
    } else {
      r.set_size(A.size(0), A.size(1));
      nx = A.size(0) * A.size(1);
      for (i = 0; i < nx; i++) {
        r[i] = 0.0;
      }

      internal::b_svd(r, U1, s, V1);
      U.set_size(U1.size(0), U1.size(1));
      nx = U1.size(0) * U1.size(1);
      for (i = 0; i < nx; i++) {
        U[i] = rtNaN;
      }

      nx = s.size(0);
      s.set_size(nx);
      for (i = 0; i < nx; i++) {
        s[i] = rtNaN;
      }

      V.set_size(V1.size(0), V1.size(1));
      nx = V1.size(0) * V1.size(1);
      for (i = 0; i < nx; i++) {
        V[i] = rtNaN;
      }
    }

    S.set_size(U.size(1), V.size(1));
    nx = U.size(1) * V.size(1);
    for (i = 0; i < nx; i++) {
      S[i] = 0.0;
    }

    i = s.size(0) - 1;
    for (k = 0; k <= i; k++) {
      S[k + S.size(0) * k] = s[k];
    }
  }
}

//
// File trailer for svd.cpp
//
// [EOF]
//
