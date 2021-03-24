//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                const double B[9]
//                ::coder::array<double, 2U> &C
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      void mtimes(const ::coder::array<double, 2U> &A, const double B[9], ::
                  coder::array<double, 2U> &C)
      {
        int inner;
        int mc;
        mc = A.size(0);
        inner = A.size(1);
        C.set_size(A.size(0), 3);
        for (int j = 0; j < 3; j++) {
          int boffset;
          int coffset;
          int i;
          coffset = j * mc;
          boffset = j * 3;
          for (i = 0; i < mc; i++) {
            C[coffset + i] = 0.0;
          }

          for (int k = 0; k < inner; k++) {
            double bkj;
            int aoffset;
            aoffset = k * A.size(0);
            bkj = B[boffset + k];
            for (i = 0; i < mc; i++) {
              int b_i;
              b_i = coffset + i;
              C[b_i] = C[b_i] + A[aoffset + i] * bkj;
            }
          }
        }
      }

      //
      // Arguments    : const ::coder::array<double, 2U> &A
      //                const double B_data[]
      //                const int B_size[2]
      //                ::coder::array<double, 2U> &C
      // Return Type  : void
      //
      void mtimes(const ::coder::array<double, 2U> &A, const double B_data[],
                  const int B_size[2], ::coder::array<double, 2U> &C)
      {
        int m;
        int n;
        m = A.size(0);
        n = B_size[1];
        C.set_size(A.size(0), B_size[1]);
        for (int j = 0; j < n; j++) {
          int boffset;
          int coffset;
          coffset = j * m;
          boffset = j * 3;
          for (int i = 0; i < m; i++) {
            C[coffset + i] = (A[i] * B_data[boffset] + A[A.size(0) + i] *
                              B_data[boffset + 1]) + A[(A.size(0) << 1) + i] *
              B_data[boffset + 2];
          }
        }
      }

      //
      // Arguments    : const double A_data[]
      //                const int A_size[2]
      //                const ::coder::array<double, 2U> &B
      //                ::coder::array<double, 2U> &C
      // Return Type  : void
      //
      void mtimes(const double A_data[], const int A_size[2], const ::coder::
                  array<double, 2U> &B, ::coder::array<double, 2U> &C)
      {
        int inner;
        int nc;
        inner = A_size[1];
        nc = B.size(1);
        C.set_size(3, B.size(1));
        for (int j = 0; j < nc; j++) {
          int boffset;
          int coffset;
          coffset = j * 3;
          boffset = j * B.size(0);
          C[coffset] = 0.0;
          C[coffset + 1] = 0.0;
          C[coffset + 2] = 0.0;
          for (int k = 0; k < inner; k++) {
            double bkj;
            int aoffset;
            aoffset = k * 3;
            bkj = B[boffset + k];
            C[coffset] = C[coffset] + A_data[aoffset] * bkj;
            C[coffset + 1] = C[coffset + 1] + A_data[aoffset + 1] * bkj;
            C[coffset + 2] = C[coffset + 2] + A_data[aoffset + 2] * bkj;
          }
        }
      }

      //
      // Arguments    : const ::coder::array<double, 2U> &A
      //                const ::coder::array<double, 2U> &B
      //                double C[9]
      // Return Type  : void
      //
      void mtimes(const ::coder::array<double, 2U> &A, const ::coder::array<
                  double, 2U> &B, double C[9])
      {
        int inner;
        inner = A.size(1);
        for (int j = 0; j < 3; j++) {
          int boffset;
          int coffset;
          coffset = j * 3;
          boffset = j * B.size(0);
          C[coffset] = 0.0;
          C[coffset + 1] = 0.0;
          C[coffset + 2] = 0.0;
          for (int k = 0; k < inner; k++) {
            double bkj;
            int aoffset;
            aoffset = k * 3;
            bkj = B[boffset + k];
            C[coffset] += A[aoffset] * bkj;
            C[coffset + 1] += A[aoffset + 1] * bkj;
            C[coffset + 2] += A[aoffset + 2] * bkj;
          }
        }
      }

      //
      // Arguments    : const double A[9]
      //                const ::coder::array<double, 2U> &B
      //                ::coder::array<double, 2U> &C
      // Return Type  : void
      //
      void mtimes(const double A[9], const ::coder::array<double, 2U> &B, ::
                  coder::array<double, 2U> &C)
      {
        int n;
        n = B.size(1);
        C.set_size(3, B.size(1));
        for (int j = 0; j < n; j++) {
          int coffset_tmp;
          coffset_tmp = j * 3;
          for (int i = 0; i < 3; i++) {
            C[coffset_tmp + i] = (A[i] * B[coffset_tmp] + A[i + 3] *
                                  B[coffset_tmp + 1]) + A[i + 6] * B[coffset_tmp
              + 2];
          }
        }
      }

      //
      // Arguments    : const ::coder::array<double, 2U> &A
      //                const ::coder::array<double, 2U> &B
      //                ::coder::array<double, 2U> &C
      // Return Type  : void
      //
      void mtimes(const ::coder::array<double, 2U> &A, const ::coder::array<
                  double, 2U> &B, ::coder::array<double, 2U> &C)
      {
        int inner;
        int mc;
        int nc;
        mc = A.size(1);
        inner = A.size(0);
        nc = B.size(1);
        C.set_size(A.size(1), B.size(1));
        for (int j = 0; j < nc; j++) {
          int boffset;
          int coffset;
          int i;
          coffset = j * mc;
          boffset = j * B.size(0);
          for (i = 0; i < mc; i++) {
            C[coffset + i] = 0.0;
          }

          for (int k = 0; k < inner; k++) {
            double bkj;
            bkj = B[boffset + k];
            for (i = 0; i < mc; i++) {
              int b_i;
              b_i = coffset + i;
              C[b_i] = C[b_i] + A[i * A.size(0) + k] * bkj;
            }
          }
        }
      }
    }
  }
}

//
// File trailer for mtimes.cpp
//
// [EOF]
//
