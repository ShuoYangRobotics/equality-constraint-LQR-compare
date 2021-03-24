//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mldivide.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                double B_data[]
//                int B_size[2]
// Return Type  : void
//
namespace coder
{
  void mldivide(const double A_data[], const int A_size[2], double B_data[], int
                B_size[2])
  {
    double b_A_data[9];
    double b_B_data[9];
    double tau_data[3];
    double wj;
    int jpvt_data[3];
    int b_A_size[2];
    int jpvt_size[2];
    int tau_size[1];
    int ix;
    int maxmn;
    int minmn;
    if ((A_size[0] == 0) || (A_size[1] == 0) || (B_size[0] == 0)) {
      B_size[0] = static_cast<signed char>(A_size[1]);
      B_size[1] = 3;
      minmn = static_cast<signed char>(A_size[1]);
      for (int i = 0; i < 3; i++) {
        for (int i1 = 0; i1 < minmn; i1++) {
          B_data[i1 + B_size[0] * i] = 0.0;
        }
      }
    } else if (A_size[0] == A_size[1]) {
      double tol;
      int LDA;
      int b_i;
      int i;
      int i1;
      int j;
      int k;
      int m;
      int mn;
      int n;
      int u0;
      u0 = A_size[0];
      n = A_size[1];
      if (u0 < n) {
        n = u0;
      }

      u0 = B_size[0];
      if (u0 < n) {
        n = u0;
      }

      LDA = A_size[0];
      minmn = A_size[0] * A_size[1];
      if (0 <= minmn - 1) {
        std::memcpy(&b_A_data[0], &A_data[0], minmn * sizeof(double));
      }

      if (n < 1) {
        minmn = 0;
      } else {
        minmn = n;
      }

      if (minmn > 0) {
        jpvt_data[0] = 1;
        maxmn = 1;
        for (k = 2; k <= minmn; k++) {
          maxmn++;
          jpvt_data[k - 1] = maxmn;
        }
      }

      if (n >= 1) {
        int rankR;
        rankR = A_size[0];
        u0 = n - 1;
        if (u0 >= n) {
          u0 = n;
        }

        for (j = 0; j < u0; j++) {
          int jj;
          int mmj_tmp;
          mmj_tmp = n - j;
          maxmn = j * (LDA + 1);
          jj = j * (rankR + 1);
          m = maxmn + 2;
          if (mmj_tmp < 1) {
            minmn = -1;
          } else {
            minmn = 0;
            if (mmj_tmp > 1) {
              ix = maxmn;
              tol = std::abs(b_A_data[jj]);
              for (k = 2; k <= mmj_tmp; k++) {
                ix++;
                wj = std::abs(b_A_data[ix]);
                if (wj > tol) {
                  minmn = k - 1;
                  tol = wj;
                }
              }
            }
          }

          if (b_A_data[jj + minmn] != 0.0) {
            if (minmn != 0) {
              minmn += j;
              jpvt_data[j] = minmn + 1;
              ix = j;
              for (k = 0; k < n; k++) {
                tol = b_A_data[ix];
                b_A_data[ix] = b_A_data[minmn];
                b_A_data[minmn] = tol;
                ix += LDA;
                minmn += LDA;
              }
            }

            i = jj + mmj_tmp;
            for (b_i = m; b_i <= i; b_i++) {
              b_A_data[b_i - 1] /= b_A_data[jj];
            }
          }

          minmn = maxmn + LDA;
          maxmn = jj + rankR;
          for (m = 0; m <= mmj_tmp - 2; m++) {
            tol = b_A_data[minmn];
            if (b_A_data[minmn] != 0.0) {
              ix = jj + 1;
              i = maxmn + 2;
              i1 = mmj_tmp + maxmn;
              for (mn = i; mn <= i1; mn++) {
                b_A_data[mn - 1] += b_A_data[ix] * -tol;
                ix++;
              }
            }

            minmn += LDA;
            maxmn += LDA;
          }
        }
      }

      LDA = A_size[0];
      m = B_size[0];
      for (b_i = 0; b_i <= n - 2; b_i++) {
        i = jpvt_data[b_i];
        if (i != b_i + 1) {
          tol = B_data[b_i];
          B_data[b_i] = B_data[i - 1];
          B_data[i - 1] = tol;
          minmn = b_i + B_size[0];
          tol = B_data[minmn];
          i1 = (i + B_size[0]) - 1;
          B_data[minmn] = B_data[i1];
          B_data[i1] = tol;
          minmn = b_i + B_size[0] * 2;
          tol = B_data[minmn];
          i = (i + B_size[0] * 2) - 1;
          B_data[minmn] = B_data[i];
          B_data[i] = tol;
        }
      }

      if (B_size[0] != 0) {
        for (j = 0; j < 3; j++) {
          minmn = m * j;
          for (k = 0; k < n; k++) {
            maxmn = LDA * k;
            i = k + minmn;
            if (B_data[i] != 0.0) {
              i1 = k + 2;
              for (b_i = i1; b_i <= n; b_i++) {
                mn = (b_i + minmn) - 1;
                B_data[mn] -= B_data[i] * b_A_data[(b_i + maxmn) - 1];
              }
            }
          }
        }
      }

      if (B_size[0] != 0) {
        for (j = 0; j < 3; j++) {
          minmn = m * j - 1;
          for (k = n; k >= 1; k--) {
            maxmn = LDA * (k - 1) - 1;
            i = k + minmn;
            if (B_data[i] != 0.0) {
              B_data[i] /= b_A_data[k + maxmn];
              for (b_i = 0; b_i <= k - 2; b_i++) {
                i1 = (b_i + minmn) + 1;
                B_data[i1] -= B_data[i] * b_A_data[(b_i + maxmn) + 1];
              }
            }
          }
        }
      }
    } else {
      double tol;
      int b_i;
      int i;
      int i1;
      int j;
      int k;
      int m;
      int mn;
      int rankR;
      int u0;
      b_A_size[0] = A_size[0];
      b_A_size[1] = A_size[1];
      minmn = A_size[0] * A_size[1];
      if (0 <= minmn - 1) {
        std::memcpy(&b_A_data[0], &A_data[0], minmn * sizeof(double));
      }

      internal::lapack::xgeqp3(b_A_data, b_A_size, tau_data, tau_size, jpvt_data,
        jpvt_size);
      rankR = 0;
      if (b_A_size[0] < b_A_size[1]) {
        minmn = b_A_size[0];
        maxmn = b_A_size[1];
      } else {
        minmn = b_A_size[1];
        maxmn = b_A_size[0];
      }

      if (minmn > 0) {
        tol = 2.2204460492503131E-15 * static_cast<double>(maxmn) * std::abs
          (b_A_data[0]);
        while ((rankR < minmn) && (!(std::abs(b_A_data[rankR + b_A_size[0] *
                  rankR]) <= tol))) {
          rankR++;
        }
      }

      ix = B_size[0];
      minmn = B_size[0] * B_size[1];
      if (0 <= minmn - 1) {
        std::memcpy(&b_B_data[0], &B_data[0], minmn * sizeof(double));
      }

      B_size[0] = static_cast<signed char>(b_A_size[1]);
      B_size[1] = 3;
      minmn = static_cast<signed char>(b_A_size[1]);
      for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < minmn; i1++) {
          B_data[i1 + B_size[0] * i] = 0.0;
        }
      }

      m = b_A_size[0];
      u0 = b_A_size[0];
      mn = b_A_size[1];
      if (u0 < mn) {
        mn = u0;
      }

      for (j = 0; j < mn; j++) {
        if (tau_data[j] != 0.0) {
          i = j + 2;
          for (k = 0; k < 3; k++) {
            maxmn = ix * k;
            minmn = j + maxmn;
            tol = b_B_data[minmn];
            wj = tol;
            for (b_i = i; b_i <= m; b_i++) {
              wj += b_A_data[(b_i + b_A_size[0] * j) - 1] * b_B_data[(b_i +
                maxmn) - 1];
            }

            wj *= tau_data[j];
            if (wj != 0.0) {
              b_B_data[minmn] = tol - wj;
              i1 = j + 2;
              for (b_i = i1; b_i <= m; b_i++) {
                minmn = (b_i + maxmn) - 1;
                b_B_data[minmn] -= b_A_data[(b_i + b_A_size[0] * j) - 1] * wj;
              }
            }
          }
        }
      }

      for (k = 0; k < 3; k++) {
        for (b_i = 0; b_i < rankR; b_i++) {
          B_data[(jpvt_data[b_i] + B_size[0] * k) - 1] = b_B_data[b_i + ix * k];
        }

        for (j = rankR; j >= 1; j--) {
          i = B_size[0] * k;
          i1 = (jpvt_data[j - 1] + i) - 1;
          mn = b_A_size[0] * (j - 1);
          B_data[i1] /= b_A_data[(j + mn) - 1];
          for (b_i = 0; b_i <= j - 2; b_i++) {
            i1 = (jpvt_data[b_i] + i) - 1;
            B_data[i1] -= B_data[(jpvt_data[j - 1] + B_size[0] * k) - 1] *
              b_A_data[b_i + mn];
          }
        }
      }
    }
  }

  //
  // Arguments    : const double A[9]
  //                const double B[9]
  //                double Y[9]
  // Return Type  : void
  //
  void mldivide(const double A[9], const double B[9], double Y[9])
  {
    double b_A[9];
    double a21;
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    double d5;
    double d6;
    double maxval;
    int r1;
    int r2;
    int r3;
    std::memcpy(&b_A[0], &A[0], 9U * sizeof(double));
    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = std::abs(A[0]);
    a21 = std::abs(A[1]);
    if (a21 > maxval) {
      maxval = a21;
      r1 = 1;
      r2 = 0;
    }

    if (std::abs(A[2]) > maxval) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }

    b_A[r2] = A[r2] / A[r1];
    b_A[r3] /= b_A[r1];
    b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
    b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
    b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
    b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
    if (std::abs(b_A[r3 + 3]) > std::abs(b_A[r2 + 3])) {
      int rtemp;
      rtemp = r2;
      r2 = r3;
      r3 = rtemp;
    }

    b_A[r3 + 3] /= b_A[r2 + 3];
    b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
    maxval = B[r1];
    a21 = B[r2] - maxval * b_A[r2];
    d = b_A[r3 + 3];
    d1 = b_A[r3 + 6];
    d2 = ((B[r3] - maxval * b_A[r3]) - a21 * d) / d1;
    Y[2] = d2;
    d3 = b_A[r1 + 6];
    maxval -= d2 * d3;
    d4 = b_A[r2 + 6];
    a21 -= d2 * d4;
    d5 = b_A[r2 + 3];
    a21 /= d5;
    Y[1] = a21;
    d6 = b_A[r1 + 3];
    maxval -= a21 * d6;
    maxval /= b_A[r1];
    Y[0] = maxval;
    maxval = B[r1 + 3];
    a21 = B[r2 + 3] - maxval * b_A[r2];
    d2 = ((B[r3 + 3] - maxval * b_A[r3]) - a21 * d) / d1;
    Y[5] = d2;
    maxval -= d2 * d3;
    a21 -= d2 * d4;
    a21 /= d5;
    Y[4] = a21;
    maxval -= a21 * d6;
    maxval /= b_A[r1];
    Y[3] = maxval;
    maxval = B[r1 + 6];
    a21 = B[r2 + 6] - maxval * b_A[r2];
    d2 = ((B[r3 + 6] - maxval * b_A[r3]) - a21 * d) / d1;
    Y[8] = d2;
    maxval -= d2 * d3;
    a21 -= d2 * d4;
    a21 /= d5;
    Y[7] = a21;
    maxval -= a21 * d6;
    maxval /= b_A[r1];
    Y[6] = maxval;
  }
}

//
// File trailer for mldivide.cpp
//
// [EOF]
//
