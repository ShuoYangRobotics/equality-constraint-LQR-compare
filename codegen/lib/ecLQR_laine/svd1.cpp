//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "svd1.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                ::coder::array<double, 2U> &U
//                double s_data[]
//                int s_size[1]
//                double V[9]
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    void b_svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U>
               &U, double s_data[], int s_size[1], double V[9])
    {
      array<double, 2U> b_A;
      array<double, 1U> work;
      double b_s_data[3];
      double e[3];
      double f;
      double nrm;
      double rt;
      double scale;
      double snorm;
      double sqds;
      double ztest;
      int ix;
      int iy;
      int jj;
      int k;
      int m;
      int n;
      int nct;
      int nctp1;
      int nmq;
      int q;
      int qjj;
      int qp1;
      int qq;
      b_A.set_size(A.size(0), 3);
      nmq = A.size(0) * A.size(1);
      for (qjj = 0; qjj < nmq; qjj++) {
        b_A[qjj] = A[qjj];
      }

      n = A.size(0);
      b_s_data[0] = 0.0;
      e[0] = 0.0;
      b_s_data[1] = 0.0;
      e[1] = 0.0;
      b_s_data[2] = 0.0;
      e[2] = 0.0;
      work.set_size(A.size(0));
      nmq = A.size(0);
      for (qjj = 0; qjj < nmq; qjj++) {
        work[qjj] = 0.0;
      }

      U.set_size(A.size(0), A.size(0));
      nmq = A.size(0) * A.size(0);
      for (qjj = 0; qjj < nmq; qjj++) {
        U[qjj] = 0.0;
      }

      std::memset(&V[0], 0, 9U * sizeof(double));
      nct = A.size(0) - 1;
      if (nct >= 3) {
        nct = 3;
      }

      nctp1 = nct + 1;
      for (q = 0; q < nct; q++) {
        boolean_T apply_transform;
        qp1 = q + 2;
        qq = (q + n * q) + 1;
        nmq = (n - q) - 1;
        apply_transform = false;
        if (q + 1 <= nct) {
          nrm = blas::xnrm2(nmq + 1, b_A, qq);
          if (nrm > 0.0) {
            apply_transform = true;
            if (b_A[qq - 1] < 0.0) {
              ztest = -nrm;
              b_s_data[q] = -nrm;
            } else {
              ztest = nrm;
              b_s_data[q] = nrm;
            }

            if (std::abs(ztest) >= 1.0020841800044864E-292) {
              nrm = 1.0 / ztest;
              qjj = qq + nmq;
              for (k = qq; k <= qjj; k++) {
                b_A[k - 1] = nrm * b_A[k - 1];
              }
            } else {
              qjj = qq + nmq;
              for (k = qq; k <= qjj; k++) {
                b_A[k - 1] = b_A[k - 1] / b_s_data[q];
              }
            }

            b_A[qq - 1] = b_A[qq - 1] + 1.0;
            b_s_data[q] = -b_s_data[q];
          } else {
            b_s_data[q] = 0.0;
          }
        }

        for (jj = qp1; jj < 4; jj++) {
          qjj = q + n * (jj - 1);
          if (apply_transform) {
            ix = qq;
            iy = qjj;
            nrm = 0.0;
            for (k = 0; k <= nmq; k++) {
              nrm += b_A[ix - 1] * b_A[iy];
              ix++;
              iy++;
            }

            nrm = -(nrm / b_A[q + b_A.size(0) * q]);
            blas::xaxpy(nmq + 1, nrm, qq, b_A, qjj + 1);
          }

          e[jj - 1] = b_A[qjj];
        }

        if (q + 1 <= nct) {
          for (ix = q + 1; ix <= n; ix++) {
            U[(ix + U.size(0) * q) - 1] = b_A[(ix + b_A.size(0) * q) - 1];
          }
        }

        if (q + 1 <= 1) {
          nrm = blas::xnrm2(e, 2);
          if (nrm == 0.0) {
            e[0] = 0.0;
          } else {
            if (e[1] < 0.0) {
              e[0] = -nrm;
            } else {
              e[0] = nrm;
            }

            nrm = e[0];
            if (std::abs(e[0]) >= 1.0020841800044864E-292) {
              nrm = 1.0 / e[0];
              for (k = qp1; k < 4; k++) {
                e[k - 1] *= nrm;
              }
            } else {
              for (k = qp1; k < 4; k++) {
                e[k - 1] /= nrm;
              }
            }

            e[1]++;
            e[0] = -e[0];
            for (ix = qp1; ix <= n; ix++) {
              work[ix - 1] = 0.0;
            }

            for (jj = qp1; jj < 4; jj++) {
              blas::xaxpy(nmq, e[jj - 1], b_A, n * (jj - 1) + 2, work, 2);
            }

            for (jj = qp1; jj < 4; jj++) {
              blas::xaxpy(nmq, -e[jj - 1] / e[1], work, 2, b_A, n * (jj - 1) + 2);
            }
          }

          for (ix = qp1; ix < 4; ix++) {
            V[ix - 1] = e[ix - 1];
          }
        }
      }

      m = 1;
      if (nct < 3) {
        b_s_data[2] = b_A[nct + b_A.size(0) * 2];
      }

      e[1] = b_A[b_A.size(0) * 2 + 1];
      e[2] = 0.0;
      if (nct + 1 <= A.size(0)) {
        for (jj = nctp1; jj <= n; jj++) {
          for (ix = 0; ix < n; ix++) {
            U[ix + U.size(0) * (jj - 1)] = 0.0;
          }

          U[(jj + U.size(0) * (jj - 1)) - 1] = 1.0;
        }
      }

      for (q = nct; q >= 1; q--) {
        qp1 = q + 1;
        nmq = n - q;
        qq = (q + n * (q - 1)) - 1;
        if (b_s_data[q - 1] != 0.0) {
          for (jj = qp1; jj <= n; jj++) {
            qjj = q + n * (jj - 1);
            nrm = 0.0;
            if (nmq + 1 >= 1) {
              ix = qq;
              iy = qjj;
              for (k = 0; k <= nmq; k++) {
                nrm += U[ix] * U[iy - 1];
                ix++;
                iy++;
              }
            }

            nrm = -(nrm / U[qq]);
            blas::b_xaxpy(nmq + 1, nrm, qq + 1, U, qjj);
          }

          for (ix = q; ix <= n; ix++) {
            U[(ix + U.size(0) * (q - 1)) - 1] = -U[(ix + U.size(0) * (q - 1)) -
              1];
          }

          U[qq] = U[qq] + 1.0;
          for (ix = 0; ix <= q - 2; ix++) {
            U[ix + U.size(0) * (q - 1)] = 0.0;
          }
        } else {
          for (ix = 0; ix < n; ix++) {
            U[ix + U.size(0) * (q - 1)] = 0.0;
          }

          U[qq] = 1.0;
        }
      }

      for (q = 2; q >= 0; q--) {
        if ((q + 1 <= 1) && (e[0] != 0.0)) {
          blas::xaxpy(-(blas::xdotc(V, V, 5) / V[1]), V, 5);
          blas::xaxpy(-(blas::xdotc(V, V, 8) / V[1]), V, 8);
        }

        V[3 * q] = 0.0;
        V[3 * q + 1] = 0.0;
        V[3 * q + 2] = 0.0;
        V[q + 3 * q] = 1.0;
      }

      nctp1 = 0;
      snorm = 0.0;
      for (q = 0; q < 3; q++) {
        ztest = b_s_data[q];
        if (ztest != 0.0) {
          rt = std::abs(ztest);
          nrm = ztest / rt;
          b_s_data[q] = rt;
          if (q + 1 < 3) {
            e[q] /= nrm;
          }

          nmq = n * q;
          qjj = nmq + n;
          for (k = nmq + 1; k <= qjj; k++) {
            U[k - 1] = nrm * U[k - 1];
          }
        }

        if (q + 1 < 3) {
          ztest = e[q];
          if (ztest != 0.0) {
            rt = std::abs(ztest);
            nrm = rt / ztest;
            e[q] = rt;
            b_s_data[q + 1] *= nrm;
            nmq = 3 * (q + 1);
            qjj = nmq + 3;
            for (k = nmq + 1; k <= qjj; k++) {
              V[k - 1] *= nrm;
            }
          }
        }

        nrm = std::abs(b_s_data[q]);
        ztest = std::abs(e[q]);
        if ((nrm > ztest) || rtIsNaN(ztest)) {
          ztest = nrm;
        }

        if ((!(snorm > ztest)) && (!rtIsNaN(ztest))) {
          snorm = ztest;
        }
      }

      while ((m + 2 > 0) && (nctp1 < 75)) {
        ix = m;
        int exitg1;
        do {
          exitg1 = 0;
          q = ix + 1;
          if (ix + 1 == 0) {
            exitg1 = 1;
          } else {
            nrm = std::abs(e[ix]);
            if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[ix]) + std::
                  abs(b_s_data[ix + 1]))) || (nrm <= 1.0020841800044864E-292) ||
                ((nctp1 > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
              e[ix] = 0.0;
              exitg1 = 1;
            } else {
              ix--;
            }
          }
        } while (exitg1 == 0);

        if (ix + 1 == m + 1) {
          nmq = 4;
        } else {
          boolean_T exitg2;
          qjj = m + 2;
          nmq = m + 2;
          exitg2 = false;
          while ((!exitg2) && (nmq >= ix + 1)) {
            qjj = nmq;
            if (nmq == ix + 1) {
              exitg2 = true;
            } else {
              nrm = 0.0;
              if (nmq < m + 2) {
                nrm = std::abs(e[nmq - 1]);
              }

              if (nmq > ix + 2) {
                nrm += std::abs(e[nmq - 2]);
              }

              ztest = std::abs(b_s_data[nmq - 1]);
              if ((ztest <= 2.2204460492503131E-16 * nrm) || (ztest <=
                   1.0020841800044864E-292)) {
                b_s_data[nmq - 1] = 0.0;
                exitg2 = true;
              } else {
                nmq--;
              }
            }
          }

          if (qjj == ix + 1) {
            nmq = 3;
          } else if (qjj == m + 2) {
            nmq = 1;
          } else {
            nmq = 2;
            q = qjj;
          }
        }

        switch (nmq) {
         case 1:
          f = e[m];
          e[m] = 0.0;
          qjj = m + 1;
          for (k = qjj; k >= q + 1; k--) {
            blas::xrotg(&b_s_data[k - 1], &f, &sqds, &scale);
            if (k > q + 1) {
              f = -scale * e[0];
              e[0] *= sqds;
            }

            blas::xrot(V, 3 * (k - 1) + 1, 3 * (m + 1) + 1, sqds, scale);
          }
          break;

         case 2:
          f = e[q - 1];
          e[q - 1] = 0.0;
          for (k = q + 1; k <= m + 2; k++) {
            blas::xrotg(&b_s_data[k - 1], &f, &sqds, &scale);
            rt = e[k - 1];
            f = -scale * rt;
            e[k - 1] = rt * sqds;
            if (n >= 1) {
              ix = n * (k - 1);
              iy = n * (q - 1);
              for (qjj = 0; qjj < n; qjj++) {
                nrm = sqds * U[ix] + scale * U[iy];
                U[iy] = sqds * U[iy] - scale * U[ix];
                U[ix] = nrm;
                iy++;
                ix++;
              }
            }
          }
          break;

         case 3:
          nmq = m + 1;
          nrm = b_s_data[m + 1];
          scale = std::abs(nrm);
          ztest = std::abs(b_s_data[m]);
          if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
            scale = ztest;
          }

          ztest = std::abs(e[m]);
          if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
            scale = ztest;
          }

          ztest = std::abs(b_s_data[q]);
          if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
            scale = ztest;
          }

          ztest = std::abs(e[q]);
          if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
            scale = ztest;
          }

          f = nrm / scale;
          nrm = b_s_data[m] / scale;
          ztest = e[m] / scale;
          sqds = b_s_data[q] / scale;
          rt = ((nrm + f) * (nrm - f) + ztest * ztest) / 2.0;
          nrm = f * ztest;
          nrm *= nrm;
          if ((rt != 0.0) || (nrm != 0.0)) {
            ztest = std::sqrt(rt * rt + nrm);
            if (rt < 0.0) {
              ztest = -ztest;
            }

            ztest = nrm / (rt + ztest);
          } else {
            ztest = 0.0;
          }

          f = (sqds + f) * (sqds - f) + ztest;
          ztest = sqds * (e[q] / scale);
          for (k = q + 1; k <= nmq; k++) {
            blas::xrotg(&f, &ztest, &sqds, &scale);
            if (k > q + 1) {
              e[0] = f;
            }

            nrm = e[k - 1];
            rt = b_s_data[k - 1];
            e[k - 1] = sqds * nrm - scale * rt;
            ztest = scale * b_s_data[k];
            b_s_data[k] *= sqds;
            blas::xrot(V, 3 * (k - 1) + 1, 3 * k + 1, sqds, scale);
            b_s_data[k - 1] = sqds * rt + scale * nrm;
            blas::xrotg(&b_s_data[k - 1], &ztest, &sqds, &scale);
            f = sqds * e[k - 1] + scale * b_s_data[k];
            b_s_data[k] = -scale * e[k - 1] + sqds * b_s_data[k];
            ztest = scale * e[k];
            e[k] *= sqds;
            if (n >= 1) {
              ix = n * (k - 1);
              iy = n * k;
              for (qjj = 0; qjj < n; qjj++) {
                nrm = sqds * U[ix] + scale * U[iy];
                U[iy] = sqds * U[iy] - scale * U[ix];
                U[ix] = nrm;
                iy++;
                ix++;
              }
            }
          }

          e[m] = f;
          nctp1++;
          break;

         default:
          if (b_s_data[q] < 0.0) {
            b_s_data[q] = -b_s_data[q];
            nmq = 3 * q;
            qjj = nmq + 3;
            for (k = nmq + 1; k <= qjj; k++) {
              V[k - 1] = -V[k - 1];
            }
          }

          qp1 = q + 1;
          while ((q + 1 < 3) && (b_s_data[q] < b_s_data[qp1])) {
            rt = b_s_data[q];
            b_s_data[q] = b_s_data[qp1];
            b_s_data[qp1] = rt;
            blas::xswap(V, 3 * q + 1, 3 * (q + 1) + 1);
            ix = n * q;
            iy = n * (q + 1);
            for (k = 0; k < n; k++) {
              nrm = U[ix];
              U[ix] = U[iy];
              U[iy] = nrm;
              ix++;
              iy++;
            }

            q = qp1;
            qp1++;
          }

          nctp1 = 0;
          m--;
          break;
        }
      }

      s_size[0] = 3;
      s_data[0] = b_s_data[0];
      s_data[1] = b_s_data[1];
      s_data[2] = b_s_data[2];
    }

    //
    // Arguments    : const ::coder::array<double, 2U> &A
    //                ::coder::array<double, 2U> &U
    //                double s_data[]
    //                int s_size[1]
    //                double V_data[]
    //                int V_size[2]
    // Return Type  : void
    //
    void b_svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U>
               &U, double s_data[], int s_size[1], double V_data[], int V_size[2])
    {
      array<double, 2U> b_A;
      array<double, 1U> work;
      double Vf[9];
      double b_s_data[3];
      double e[3];
      double f;
      double nrm;
      double rt;
      double scale;
      double snorm;
      double sqds;
      double ztest;
      int ix;
      int iy;
      int jj;
      int k;
      int m;
      int n;
      int nct;
      int nmq;
      int q;
      int qjj;
      int qp1;
      int qq;
      b_A.set_size(A.size(0), 3);
      nmq = A.size(0) * A.size(1);
      for (qjj = 0; qjj < nmq; qjj++) {
        b_A[qjj] = A[qjj];
      }

      n = A.size(0);
      b_s_data[0] = 0.0;
      e[0] = 0.0;
      b_s_data[1] = 0.0;
      e[1] = 0.0;
      b_s_data[2] = 0.0;
      e[2] = 0.0;
      work.set_size(A.size(0));
      nmq = A.size(0);
      for (qjj = 0; qjj < nmq; qjj++) {
        work[qjj] = 0.0;
      }

      U.set_size(A.size(0), 3);
      nmq = A.size(0) * 3;
      for (qjj = 0; qjj < nmq; qjj++) {
        U[qjj] = 0.0;
      }

      std::memset(&Vf[0], 0, 9U * sizeof(double));
      nct = A.size(0) - 1;
      if (nct >= 3) {
        nct = 3;
      }

      for (q = 0; q < nct; q++) {
        boolean_T apply_transform;
        qp1 = q + 2;
        qq = (q + n * q) + 1;
        nmq = (n - q) - 1;
        apply_transform = false;
        if (q + 1 <= nct) {
          nrm = blas::xnrm2(nmq + 1, b_A, qq);
          if (nrm > 0.0) {
            apply_transform = true;
            if (b_A[qq - 1] < 0.0) {
              ztest = -nrm;
              b_s_data[q] = -nrm;
            } else {
              ztest = nrm;
              b_s_data[q] = nrm;
            }

            if (std::abs(ztest) >= 1.0020841800044864E-292) {
              nrm = 1.0 / ztest;
              qjj = qq + nmq;
              for (k = qq; k <= qjj; k++) {
                b_A[k - 1] = nrm * b_A[k - 1];
              }
            } else {
              qjj = qq + nmq;
              for (k = qq; k <= qjj; k++) {
                b_A[k - 1] = b_A[k - 1] / b_s_data[q];
              }
            }

            b_A[qq - 1] = b_A[qq - 1] + 1.0;
            b_s_data[q] = -b_s_data[q];
          } else {
            b_s_data[q] = 0.0;
          }
        }

        for (jj = qp1; jj < 4; jj++) {
          qjj = q + n * (jj - 1);
          if (apply_transform) {
            ix = qq;
            iy = qjj;
            nrm = 0.0;
            for (k = 0; k <= nmq; k++) {
              nrm += b_A[ix - 1] * b_A[iy];
              ix++;
              iy++;
            }

            nrm = -(nrm / b_A[q + b_A.size(0) * q]);
            blas::xaxpy(nmq + 1, nrm, qq, b_A, qjj + 1);
          }

          e[jj - 1] = b_A[qjj];
        }

        if (q + 1 <= nct) {
          for (ix = q + 1; ix <= n; ix++) {
            U[(ix + U.size(0) * q) - 1] = b_A[(ix + b_A.size(0) * q) - 1];
          }
        }

        if (q + 1 <= 1) {
          nrm = blas::xnrm2(e, 2);
          if (nrm == 0.0) {
            e[0] = 0.0;
          } else {
            if (e[1] < 0.0) {
              e[0] = -nrm;
            } else {
              e[0] = nrm;
            }

            nrm = e[0];
            if (std::abs(e[0]) >= 1.0020841800044864E-292) {
              nrm = 1.0 / e[0];
              for (k = qp1; k < 4; k++) {
                e[k - 1] *= nrm;
              }
            } else {
              for (k = qp1; k < 4; k++) {
                e[k - 1] /= nrm;
              }
            }

            e[1]++;
            e[0] = -e[0];
            for (ix = qp1; ix <= n; ix++) {
              work[ix - 1] = 0.0;
            }

            for (jj = qp1; jj < 4; jj++) {
              blas::xaxpy(nmq, e[jj - 1], b_A, n * (jj - 1) + 2, work, 2);
            }

            for (jj = qp1; jj < 4; jj++) {
              blas::xaxpy(nmq, -e[jj - 1] / e[1], work, 2, b_A, n * (jj - 1) + 2);
            }
          }

          for (ix = qp1; ix < 4; ix++) {
            Vf[ix - 1] = e[ix - 1];
          }
        }
      }

      m = 1;
      if (nct < 3) {
        b_s_data[2] = b_A[nct + b_A.size(0) * 2];
      }

      e[1] = b_A[b_A.size(0) * 2 + 1];
      e[2] = 0.0;
      if (nct + 1 <= 3) {
        for (ix = 0; ix < n; ix++) {
          U[ix + U.size(0) * 2] = 0.0;
        }

        U[U.size(0) * 2 + 2] = 1.0;
      }

      for (q = nct; q >= 1; q--) {
        qp1 = q + 1;
        nmq = n - q;
        qq = (q + n * (q - 1)) - 1;
        if (b_s_data[q - 1] != 0.0) {
          for (jj = qp1; jj < 4; jj++) {
            qjj = q + n * (jj - 1);
            nrm = 0.0;
            if (nmq + 1 >= 1) {
              ix = qq;
              iy = qjj;
              for (k = 0; k <= nmq; k++) {
                nrm += U[ix] * U[iy - 1];
                ix++;
                iy++;
              }
            }

            nrm = -(nrm / U[qq]);
            blas::b_xaxpy(nmq + 1, nrm, qq + 1, U, qjj);
          }

          for (ix = q; ix <= n; ix++) {
            U[(ix + U.size(0) * (q - 1)) - 1] = -U[(ix + U.size(0) * (q - 1)) -
              1];
          }

          U[qq] = U[qq] + 1.0;
          for (ix = 0; ix <= q - 2; ix++) {
            U[ix + U.size(0) * (q - 1)] = 0.0;
          }
        } else {
          for (ix = 0; ix < n; ix++) {
            U[ix + U.size(0) * (q - 1)] = 0.0;
          }

          U[qq] = 1.0;
        }
      }

      for (q = 2; q >= 0; q--) {
        if ((q + 1 <= 1) && (e[0] != 0.0)) {
          blas::xaxpy(-(blas::xdotc(Vf, Vf, 5) / Vf[1]), Vf, 5);
          blas::xaxpy(-(blas::xdotc(Vf, Vf, 8) / Vf[1]), Vf, 8);
        }

        Vf[3 * q] = 0.0;
        Vf[3 * q + 1] = 0.0;
        Vf[3 * q + 2] = 0.0;
        Vf[q + 3 * q] = 1.0;
      }

      nct = 0;
      snorm = 0.0;
      for (q = 0; q < 3; q++) {
        ztest = b_s_data[q];
        if (ztest != 0.0) {
          rt = std::abs(ztest);
          nrm = ztest / rt;
          b_s_data[q] = rt;
          if (q + 1 < 3) {
            e[q] /= nrm;
          }

          nmq = n * q;
          qjj = nmq + n;
          for (k = nmq + 1; k <= qjj; k++) {
            U[k - 1] = nrm * U[k - 1];
          }
        }

        if (q + 1 < 3) {
          ztest = e[q];
          if (ztest != 0.0) {
            rt = std::abs(ztest);
            nrm = rt / ztest;
            e[q] = rt;
            b_s_data[q + 1] *= nrm;
            nmq = 3 * (q + 1);
            qjj = nmq + 3;
            for (k = nmq + 1; k <= qjj; k++) {
              Vf[k - 1] *= nrm;
            }
          }
        }

        nrm = std::abs(b_s_data[q]);
        ztest = std::abs(e[q]);
        if ((nrm > ztest) || rtIsNaN(ztest)) {
          ztest = nrm;
        }

        if ((!(snorm > ztest)) && (!rtIsNaN(ztest))) {
          snorm = ztest;
        }
      }

      while ((m + 2 > 0) && (nct < 75)) {
        ix = m;
        int exitg1;
        do {
          exitg1 = 0;
          q = ix + 1;
          if (ix + 1 == 0) {
            exitg1 = 1;
          } else {
            nrm = std::abs(e[ix]);
            if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[ix]) + std::
                  abs(b_s_data[ix + 1]))) || (nrm <= 1.0020841800044864E-292) ||
                ((nct > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
              e[ix] = 0.0;
              exitg1 = 1;
            } else {
              ix--;
            }
          }
        } while (exitg1 == 0);

        if (ix + 1 == m + 1) {
          nmq = 4;
        } else {
          boolean_T exitg2;
          qjj = m + 2;
          nmq = m + 2;
          exitg2 = false;
          while ((!exitg2) && (nmq >= ix + 1)) {
            qjj = nmq;
            if (nmq == ix + 1) {
              exitg2 = true;
            } else {
              nrm = 0.0;
              if (nmq < m + 2) {
                nrm = std::abs(e[nmq - 1]);
              }

              if (nmq > ix + 2) {
                nrm += std::abs(e[nmq - 2]);
              }

              ztest = std::abs(b_s_data[nmq - 1]);
              if ((ztest <= 2.2204460492503131E-16 * nrm) || (ztest <=
                   1.0020841800044864E-292)) {
                b_s_data[nmq - 1] = 0.0;
                exitg2 = true;
              } else {
                nmq--;
              }
            }
          }

          if (qjj == ix + 1) {
            nmq = 3;
          } else if (qjj == m + 2) {
            nmq = 1;
          } else {
            nmq = 2;
            q = qjj;
          }
        }

        switch (nmq) {
         case 1:
          f = e[m];
          e[m] = 0.0;
          qjj = m + 1;
          for (k = qjj; k >= q + 1; k--) {
            blas::xrotg(&b_s_data[k - 1], &f, &sqds, &scale);
            if (k > q + 1) {
              f = -scale * e[0];
              e[0] *= sqds;
            }

            blas::xrot(Vf, 3 * (k - 1) + 1, 3 * (m + 1) + 1, sqds, scale);
          }
          break;

         case 2:
          f = e[q - 1];
          e[q - 1] = 0.0;
          for (k = q + 1; k <= m + 2; k++) {
            blas::xrotg(&b_s_data[k - 1], &f, &sqds, &scale);
            rt = e[k - 1];
            f = -scale * rt;
            e[k - 1] = rt * sqds;
            if (n >= 1) {
              ix = n * (k - 1);
              iy = n * (q - 1);
              for (qjj = 0; qjj < n; qjj++) {
                nrm = sqds * U[ix] + scale * U[iy];
                U[iy] = sqds * U[iy] - scale * U[ix];
                U[ix] = nrm;
                iy++;
                ix++;
              }
            }
          }
          break;

         case 3:
          nmq = m + 1;
          nrm = b_s_data[m + 1];
          scale = std::abs(nrm);
          ztest = std::abs(b_s_data[m]);
          if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
            scale = ztest;
          }

          ztest = std::abs(e[m]);
          if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
            scale = ztest;
          }

          ztest = std::abs(b_s_data[q]);
          if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
            scale = ztest;
          }

          ztest = std::abs(e[q]);
          if ((!(scale > ztest)) && (!rtIsNaN(ztest))) {
            scale = ztest;
          }

          f = nrm / scale;
          nrm = b_s_data[m] / scale;
          ztest = e[m] / scale;
          sqds = b_s_data[q] / scale;
          rt = ((nrm + f) * (nrm - f) + ztest * ztest) / 2.0;
          nrm = f * ztest;
          nrm *= nrm;
          if ((rt != 0.0) || (nrm != 0.0)) {
            ztest = std::sqrt(rt * rt + nrm);
            if (rt < 0.0) {
              ztest = -ztest;
            }

            ztest = nrm / (rt + ztest);
          } else {
            ztest = 0.0;
          }

          f = (sqds + f) * (sqds - f) + ztest;
          ztest = sqds * (e[q] / scale);
          for (k = q + 1; k <= nmq; k++) {
            blas::xrotg(&f, &ztest, &sqds, &scale);
            if (k > q + 1) {
              e[0] = f;
            }

            nrm = e[k - 1];
            rt = b_s_data[k - 1];
            e[k - 1] = sqds * nrm - scale * rt;
            ztest = scale * b_s_data[k];
            b_s_data[k] *= sqds;
            blas::xrot(Vf, 3 * (k - 1) + 1, 3 * k + 1, sqds, scale);
            b_s_data[k - 1] = sqds * rt + scale * nrm;
            blas::xrotg(&b_s_data[k - 1], &ztest, &sqds, &scale);
            f = sqds * e[k - 1] + scale * b_s_data[k];
            b_s_data[k] = -scale * e[k - 1] + sqds * b_s_data[k];
            ztest = scale * e[k];
            e[k] *= sqds;
            if (n >= 1) {
              ix = n * (k - 1);
              iy = n * k;
              for (qjj = 0; qjj < n; qjj++) {
                nrm = sqds * U[ix] + scale * U[iy];
                U[iy] = sqds * U[iy] - scale * U[ix];
                U[ix] = nrm;
                iy++;
                ix++;
              }
            }
          }

          e[m] = f;
          nct++;
          break;

         default:
          if (b_s_data[q] < 0.0) {
            b_s_data[q] = -b_s_data[q];
            nmq = 3 * q;
            qjj = nmq + 3;
            for (k = nmq + 1; k <= qjj; k++) {
              Vf[k - 1] = -Vf[k - 1];
            }
          }

          qp1 = q + 1;
          while ((q + 1 < 3) && (b_s_data[q] < b_s_data[qp1])) {
            rt = b_s_data[q];
            b_s_data[q] = b_s_data[qp1];
            b_s_data[qp1] = rt;
            blas::xswap(Vf, 3 * q + 1, 3 * (q + 1) + 1);
            ix = n * q;
            iy = n * (q + 1);
            for (k = 0; k < n; k++) {
              nrm = U[ix];
              U[ix] = U[iy];
              U[iy] = nrm;
              ix++;
              iy++;
            }

            q = qp1;
            qp1++;
          }

          nct = 0;
          m--;
          break;
        }
      }

      s_size[0] = 3;
      V_size[0] = 3;
      V_size[1] = 3;
      for (k = 0; k < 3; k++) {
        s_data[k] = b_s_data[k];
        V_data[3 * k] = Vf[3 * k];
        qjj = 3 * k + 1;
        V_data[qjj] = Vf[qjj];
        qjj = 3 * k + 2;
        V_data[qjj] = Vf[qjj];
      }
    }

    //
    // Arguments    : const ::coder::array<double, 2U> &A
    //                ::coder::array<double, 2U> &U
    //                ::coder::array<double, 1U> &s
    //                ::coder::array<double, 2U> &V
    // Return Type  : void
    //
    void b_svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U>
               &U, ::coder::array<double, 1U> &s, ::coder::array<double, 2U> &V)
    {
      array<double, 2U> b_A;
      array<double, 1U> b_s;
      array<double, 1U> e;
      array<double, 1U> work;
      double nrm;
      double r;
      double scale;
      double sm;
      int i;
      int k;
      int minnp;
      int n;
      int ns;
      int p;
      int qjj;
      b_A.set_size(A.size(0), A.size(1));
      ns = A.size(0) * A.size(1);
      for (i = 0; i < ns; i++) {
        b_A[i] = A[i];
      }

      n = A.size(0);
      p = A.size(1);
      qjj = A.size(0) + 1;
      ns = A.size(1);
      if (qjj < ns) {
        ns = qjj;
      }

      qjj = A.size(0);
      minnp = A.size(1);
      if (qjj < minnp) {
        minnp = qjj;
      }

      b_s.set_size(ns);
      for (i = 0; i < ns; i++) {
        b_s[i] = 0.0;
      }

      e.set_size(A.size(1));
      ns = A.size(1);
      for (i = 0; i < ns; i++) {
        e[i] = 0.0;
      }

      work.set_size(A.size(0));
      ns = A.size(0);
      for (i = 0; i < ns; i++) {
        work[i] = 0.0;
      }

      U.set_size(A.size(0), A.size(0));
      ns = A.size(0) * A.size(0);
      for (i = 0; i < ns; i++) {
        U[i] = 0.0;
      }

      V.set_size(A.size(1), A.size(1));
      ns = A.size(1) * A.size(1);
      for (i = 0; i < ns; i++) {
        V[i] = 0.0;
      }

      if ((A.size(0) == 0) || (A.size(1) == 0)) {
        int ii;
        i = A.size(0);
        for (ii = 0; ii < i; ii++) {
          U[ii + U.size(0) * ii] = 1.0;
        }

        i = A.size(1);
        for (ii = 0; ii < i; ii++) {
          V[ii + V.size(0) * ii] = 1.0;
        }
      } else {
        double snorm;
        int ii;
        int ix;
        int iy;
        int jj;
        int m;
        int nct;
        int nctp1;
        int nmq;
        int nrt;
        int q;
        int qp1;
        int qq;
        if (A.size(1) > 2) {
          qjj = A.size(1) - 2;
        } else {
          qjj = 0;
        }

        nrt = A.size(0);
        if (qjj < nrt) {
          nrt = qjj;
        }

        if (A.size(0) > 1) {
          qjj = A.size(0) - 1;
        } else {
          qjj = 0;
        }

        nct = A.size(1);
        if (qjj < nct) {
          nct = qjj;
        }

        nctp1 = nct + 1;
        if (nct > nrt) {
          i = nct;
        } else {
          i = nrt;
        }

        for (q = 0; q < i; q++) {
          boolean_T apply_transform;
          qp1 = q + 2;
          qq = (q + n * q) + 1;
          nmq = (n - q) - 1;
          apply_transform = false;
          if (q + 1 <= nct) {
            nrm = blas::b_xnrm2(nmq + 1, b_A, qq);
            if (nrm > 0.0) {
              apply_transform = true;
              if (b_A[qq - 1] < 0.0) {
                r = -nrm;
                b_s[q] = -nrm;
              } else {
                r = nrm;
                b_s[q] = nrm;
              }

              if (std::abs(r) >= 1.0020841800044864E-292) {
                nrm = 1.0 / r;
                ns = qq + nmq;
                for (k = qq; k <= ns; k++) {
                  b_A[k - 1] = nrm * b_A[k - 1];
                }
              } else {
                ns = qq + nmq;
                for (k = qq; k <= ns; k++) {
                  b_A[k - 1] = b_A[k - 1] / b_s[q];
                }
              }

              b_A[qq - 1] = b_A[qq - 1] + 1.0;
              b_s[q] = -b_s[q];
            } else {
              b_s[q] = 0.0;
            }
          }

          for (jj = qp1; jj <= p; jj++) {
            qjj = q + n * (jj - 1);
            if (apply_transform) {
              nrm = 0.0;
              if (nmq + 1 >= 1) {
                ix = qq;
                iy = qjj;
                for (k = 0; k <= nmq; k++) {
                  nrm += b_A[ix - 1] * b_A[iy];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / b_A[q + b_A.size(0) * q]);
              blas::b_xaxpy(nmq + 1, nrm, qq, b_A, qjj + 1);
            }

            e[jj - 1] = b_A[qjj];
          }

          if (q + 1 <= nct) {
            for (ii = q + 1; ii <= n; ii++) {
              U[(ii + U.size(0) * q) - 1] = b_A[(ii + b_A.size(0) * q) - 1];
            }
          }

          if (q + 1 <= nrt) {
            qq = p - q;
            nrm = blas::xnrm2(qq - 1, e, q + 2);
            if (nrm == 0.0) {
              e[q] = 0.0;
            } else {
              if (e[q + 1] < 0.0) {
                e[q] = -nrm;
              } else {
                e[q] = nrm;
              }

              nrm = e[q];
              if (std::abs(e[q]) >= 1.0020841800044864E-292) {
                nrm = 1.0 / e[q];
                ns = q + qq;
                for (k = qp1; k <= ns; k++) {
                  e[k - 1] = nrm * e[k - 1];
                }
              } else {
                ns = q + qq;
                for (k = qp1; k <= ns; k++) {
                  e[k - 1] = e[k - 1] / nrm;
                }
              }

              e[q + 1] = e[q + 1] + 1.0;
              e[q] = -e[q];
              if (q + 2 <= n) {
                for (ii = qp1; ii <= n; ii++) {
                  work[ii - 1] = 0.0;
                }

                for (jj = qp1; jj <= p; jj++) {
                  blas::xaxpy(nmq, e[jj - 1], b_A, (q + n * (jj - 1)) + 2, work,
                              q + 2);
                }

                for (jj = qp1; jj <= p; jj++) {
                  blas::xaxpy(nmq, -e[jj - 1] / e[q + 1], work, q + 2, b_A, (q +
                    n * (jj - 1)) + 2);
                }
              }
            }

            for (ii = qp1; ii <= p; ii++) {
              V[(ii + V.size(0) * q) - 1] = e[ii - 1];
            }
          }
        }

        if (A.size(1) < A.size(0) + 1) {
          m = A.size(1) - 1;
        } else {
          m = A.size(0);
        }

        if (nct < A.size(1)) {
          b_s[nct] = b_A[nct + b_A.size(0) * nct];
        }

        if (A.size(0) < m + 1) {
          b_s[m] = 0.0;
        }

        if (nrt + 1 < m + 1) {
          e[nrt] = b_A[nrt + b_A.size(0) * m];
        }

        e[m] = 0.0;
        if (nct + 1 <= A.size(0)) {
          for (jj = nctp1; jj <= n; jj++) {
            for (ii = 0; ii < n; ii++) {
              U[ii + U.size(0) * (jj - 1)] = 0.0;
            }

            U[(jj + U.size(0) * (jj - 1)) - 1] = 1.0;
          }
        }

        for (q = nct; q >= 1; q--) {
          qp1 = q + 1;
          ns = n - q;
          qq = (q + n * (q - 1)) - 1;
          if (b_s[q - 1] != 0.0) {
            for (jj = qp1; jj <= n; jj++) {
              qjj = q + n * (jj - 1);
              nrm = 0.0;
              if (ns + 1 >= 1) {
                ix = qq;
                iy = qjj;
                for (k = 0; k <= ns; k++) {
                  nrm += U[ix] * U[iy - 1];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / U[qq]);
              blas::b_xaxpy(ns + 1, nrm, qq + 1, U, qjj);
            }

            for (ii = q; ii <= n; ii++) {
              U[(ii + U.size(0) * (q - 1)) - 1] = -U[(ii + U.size(0) * (q - 1))
                - 1];
            }

            U[qq] = U[qq] + 1.0;
            for (ii = 0; ii <= q - 2; ii++) {
              U[ii + U.size(0) * (q - 1)] = 0.0;
            }
          } else {
            for (ii = 0; ii < n; ii++) {
              U[ii + U.size(0) * (q - 1)] = 0.0;
            }

            U[qq] = 1.0;
          }
        }

        for (q = p; q >= 1; q--) {
          if ((q <= nrt) && (e[q - 1] != 0.0)) {
            qp1 = q + 1;
            qq = p - q;
            ns = q + p * (q - 1);
            for (jj = qp1; jj <= p; jj++) {
              qjj = q + p * (jj - 1);
              nrm = 0.0;
              if (qq >= 1) {
                ix = ns;
                iy = qjj;
                for (k = 0; k < qq; k++) {
                  nrm += V[ix] * V[iy];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / V[ns]);
              blas::b_xaxpy(qq, nrm, ns + 1, V, qjj + 1);
            }
          }

          for (ii = 0; ii < p; ii++) {
            V[ii + V.size(0) * (q - 1)] = 0.0;
          }

          V[(q + V.size(0) * (q - 1)) - 1] = 1.0;
        }

        nmq = m;
        qq = 0;
        snorm = 0.0;
        for (q = 0; q <= m; q++) {
          if (b_s[q] != 0.0) {
            nrm = std::abs(b_s[q]);
            r = b_s[q] / nrm;
            b_s[q] = nrm;
            if (q + 1 < m + 1) {
              e[q] = e[q] / r;
            }

            if (q + 1 <= n) {
              ns = n * q;
              i = ns + n;
              for (k = ns + 1; k <= i; k++) {
                U[k - 1] = r * U[k - 1];
              }
            }
          }

          if ((q + 1 < m + 1) && (e[q] != 0.0)) {
            nrm = std::abs(e[q]);
            r = nrm / e[q];
            e[q] = nrm;
            b_s[q + 1] = b_s[q + 1] * r;
            ns = p * (q + 1);
            i = ns + p;
            for (k = ns + 1; k <= i; k++) {
              V[k - 1] = r * V[k - 1];
            }
          }

          nrm = std::abs(b_s[q]);
          r = std::abs(e[q]);
          if ((nrm > r) || rtIsNaN(r)) {
            r = nrm;
          }

          if ((!(snorm > r)) && (!rtIsNaN(r))) {
            snorm = r;
          }
        }

        while ((m + 1 > 0) && (qq < 75)) {
          boolean_T exitg1;
          ii = m;
          exitg1 = false;
          while (!(exitg1 || (ii == 0))) {
            nrm = std::abs(e[ii - 1]);
            if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s[ii - 1]) + std::
                  abs(b_s[ii]))) || (nrm <= 1.0020841800044864E-292) || ((qq >
                  20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
              e[ii - 1] = 0.0;
              exitg1 = true;
            } else {
              ii--;
            }
          }

          if (ii == m) {
            ns = 4;
          } else {
            qjj = m + 1;
            ns = m + 1;
            exitg1 = false;
            while ((!exitg1) && (ns >= ii)) {
              qjj = ns;
              if (ns == ii) {
                exitg1 = true;
              } else {
                nrm = 0.0;
                if (ns < m + 1) {
                  nrm = std::abs(e[ns - 1]);
                }

                if (ns > ii + 1) {
                  nrm += std::abs(e[ns - 2]);
                }

                r = std::abs(b_s[ns - 1]);
                if ((r <= 2.2204460492503131E-16 * nrm) || (r <=
                     1.0020841800044864E-292)) {
                  b_s[ns - 1] = 0.0;
                  exitg1 = true;
                } else {
                  ns--;
                }
              }
            }

            if (qjj == ii) {
              ns = 3;
            } else if (qjj == m + 1) {
              ns = 1;
            } else {
              ns = 2;
              ii = qjj;
            }
          }

          switch (ns) {
           case 1:
            {
              r = e[m - 1];
              e[m - 1] = 0.0;
              for (k = m; k >= ii + 1; k--) {
                blas::xrotg(&b_s[k - 1], &r, &sm, &scale);
                if (k > ii + 1) {
                  double b;
                  b = e[k - 2];
                  r = -scale * b;
                  e[k - 2] = b * sm;
                }

                if (p >= 1) {
                  ix = p * (k - 1);
                  iy = p * m;
                  for (ns = 0; ns < p; ns++) {
                    double temp;
                    temp = sm * V[ix] + scale * V[iy];
                    V[iy] = sm * V[iy] - scale * V[ix];
                    V[ix] = temp;
                    iy++;
                    ix++;
                  }
                }
              }
            }
            break;

           case 2:
            {
              r = e[ii - 1];
              e[ii - 1] = 0.0;
              for (k = ii + 1; k <= m + 1; k++) {
                double b;
                blas::xrotg(&b_s[k - 1], &r, &sm, &scale);
                b = e[k - 1];
                r = -scale * b;
                e[k - 1] = b * sm;
                if (n >= 1) {
                  ix = n * (k - 1);
                  iy = n * (ii - 1);
                  for (ns = 0; ns < n; ns++) {
                    double temp;
                    temp = sm * U[ix] + scale * U[iy];
                    U[iy] = sm * U[iy] - scale * U[ix];
                    U[ix] = temp;
                    iy++;
                    ix++;
                  }
                }
              }
            }
            break;

           case 3:
            {
              double b;
              double temp;
              scale = std::abs(b_s[m]);
              nrm = b_s[m - 1];
              r = std::abs(nrm);
              if ((!(scale > r)) && (!rtIsNaN(r))) {
                scale = r;
              }

              b = e[m - 1];
              r = std::abs(b);
              if ((!(scale > r)) && (!rtIsNaN(r))) {
                scale = r;
              }

              r = std::abs(b_s[ii]);
              if ((!(scale > r)) && (!rtIsNaN(r))) {
                scale = r;
              }

              r = std::abs(e[ii]);
              if ((!(scale > r)) && (!rtIsNaN(r))) {
                scale = r;
              }

              sm = b_s[m] / scale;
              nrm /= scale;
              r = b / scale;
              temp = b_s[ii] / scale;
              b = ((nrm + sm) * (nrm - sm) + r * r) / 2.0;
              nrm = sm * r;
              nrm *= nrm;
              if ((b != 0.0) || (nrm != 0.0)) {
                r = std::sqrt(b * b + nrm);
                if (b < 0.0) {
                  r = -r;
                }

                r = nrm / (b + r);
              } else {
                r = 0.0;
              }

              r += (temp + sm) * (temp - sm);
              nrm = temp * (e[ii] / scale);
              for (k = ii + 1; k <= m; k++) {
                blas::xrotg(&r, &nrm, &sm, &scale);
                if (k > ii + 1) {
                  e[k - 2] = r;
                }

                b = e[k - 1];
                nrm = b_s[k - 1];
                e[k - 1] = sm * b - scale * nrm;
                r = scale * b_s[k];
                b_s[k] = b_s[k] * sm;
                if (p >= 1) {
                  ix = p * (k - 1);
                  iy = p * k;
                  for (ns = 0; ns < p; ns++) {
                    temp = sm * V[ix] + scale * V[iy];
                    V[iy] = sm * V[iy] - scale * V[ix];
                    V[ix] = temp;
                    iy++;
                    ix++;
                  }
                }

                b_s[k - 1] = sm * nrm + scale * b;
                blas::xrotg(&b_s[k - 1], &r, &sm, &scale);
                r = sm * e[k - 1] + scale * b_s[k];
                b_s[k] = -scale * e[k - 1] + sm * b_s[k];
                nrm = scale * e[k];
                e[k] = e[k] * sm;
                if ((k < n) && (n >= 1)) {
                  ix = n * (k - 1);
                  iy = n * k;
                  for (ns = 0; ns < n; ns++) {
                    temp = sm * U[ix] + scale * U[iy];
                    U[iy] = sm * U[iy] - scale * U[ix];
                    U[ix] = temp;
                    iy++;
                    ix++;
                  }
                }
              }

              e[m - 1] = r;
              qq++;
            }
            break;

           default:
            {
              if (b_s[ii] < 0.0) {
                b_s[ii] = -b_s[ii];
                ns = p * ii;
                i = ns + p;
                for (k = ns + 1; k <= i; k++) {
                  V[k - 1] = -V[k - 1];
                }
              }

              qp1 = ii + 1;
              while ((ii + 1 < nmq + 1) && (b_s[ii] < b_s[qp1])) {
                double temp;
                nrm = b_s[ii];
                b_s[ii] = b_s[qp1];
                b_s[qp1] = nrm;
                if (ii + 1 < p) {
                  ix = p * ii;
                  iy = p * (ii + 1);
                  for (k = 0; k < p; k++) {
                    temp = V[ix];
                    V[ix] = V[iy];
                    V[iy] = temp;
                    ix++;
                    iy++;
                  }
                }

                if (ii + 1 < n) {
                  ix = n * ii;
                  iy = n * (ii + 1);
                  for (k = 0; k < n; k++) {
                    temp = U[ix];
                    U[ix] = U[iy];
                    U[iy] = temp;
                    ix++;
                    iy++;
                  }
                }

                ii = qp1;
                qp1++;
              }

              qq = 0;
              m--;
            }
            break;
          }
        }
      }

      s.set_size(minnp);
      for (k = 0; k < minnp; k++) {
        s[k] = b_s[k];
      }
    }

    //
    // Arguments    : const ::coder::array<double, 2U> &A
    //                ::coder::array<double, 2U> &U
    //                double s_data[]
    //                int s_size[1]
    //                double V_data[]
    //                int V_size[2]
    // Return Type  : void
    //
    void c_svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 2U>
               &U, double s_data[], int s_size[1], double V_data[], int V_size[2])
    {
      array<double, 2U> b_A;
      array<double, 2U> x;
      array<double, 1U> b_e_data;
      array<double, 1U> b_x;
      array<double, 1U> work;
      double Vf_data[9];
      double b_s_data[3];
      double e_data[3];
      double nrm;
      double rt;
      double scale;
      double sqds;
      int Vf_size_idx_0;
      int Vf_size_idx_1;
      int i;
      int iter;
      int k;
      int loop_ub;
      int n;
      int nctp1;
      int p;
      b_A.set_size(A.size(0), A.size(1));
      loop_ub = A.size(0) * A.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_A[i] = A[i];
      }

      n = A.size(0);
      p = A.size(1);
      loop_ub = A.size(1);
      if (0 <= loop_ub - 1) {
        std::memset(&b_s_data[0], 0, loop_ub * sizeof(double));
      }

      iter = A.size(1);
      loop_ub = A.size(1);
      if (0 <= loop_ub - 1) {
        std::memset(&e_data[0], 0, loop_ub * sizeof(double));
      }

      work.set_size(A.size(0));
      loop_ub = A.size(0);
      for (i = 0; i < loop_ub; i++) {
        work[i] = 0.0;
      }

      U.set_size(A.size(0), A.size(1));
      loop_ub = A.size(0) * A.size(1);
      for (i = 0; i < loop_ub; i++) {
        U[i] = 0.0;
      }

      Vf_size_idx_0 = A.size(1);
      Vf_size_idx_1 = A.size(1);
      loop_ub = A.size(1) * A.size(1);
      if (0 <= loop_ub - 1) {
        std::memset(&Vf_data[0], 0, loop_ub * sizeof(double));
      }

      if (A.size(1) == 0) {
        for (int qjj = 0; qjj < p; qjj++) {
          U[qjj + U.size(0) * qjj] = 1.0;
        }
      } else {
        double snorm;
        int ix;
        int iy;
        int jj;
        int m;
        int nct;
        int nmq;
        int q;
        int qjj;
        int qp1;
        int qq;
        int y;
        y = (A.size(1) > 2);
        nctp1 = A.size(0) - 1;
        nct = A.size(1);
        if (nctp1 < nct) {
          nct = nctp1;
        }

        nctp1 = nct + 1;
        if (nct > y) {
          i = nct;
        } else {
          i = y;
        }

        for (q = 0; q < i; q++) {
          boolean_T apply_transform;
          qp1 = q + 2;
          qq = (q + n * q) + 1;
          nmq = (n - q) - 1;
          apply_transform = false;
          if (q + 1 <= nct) {
            nrm = blas::b_xnrm2(nmq + 1, b_A, qq);
            if (nrm > 0.0) {
              apply_transform = true;
              if (b_A[qq - 1] < 0.0) {
                b_s_data[q] = -nrm;
              } else {
                b_s_data[q] = nrm;
              }

              x.set_size(b_A.size(0), b_A.size(1));
              loop_ub = b_A.size(0) * b_A.size(1);
              for (ix = 0; ix < loop_ub; ix++) {
                x[ix] = b_A[ix];
              }

              nrm = b_s_data[q];
              if (std::abs(nrm) >= 1.0020841800044864E-292) {
                nrm = 1.0 / nrm;
                ix = qq + nmq;
                for (k = qq; k <= ix; k++) {
                  x[k - 1] = nrm * x[k - 1];
                }
              } else {
                ix = qq + nmq;
                for (k = qq; k <= ix; k++) {
                  x[k - 1] = x[k - 1] / b_s_data[q];
                }
              }

              b_A.set_size(x.size(0), x.size(1));
              loop_ub = x.size(0) * x.size(1);
              for (ix = 0; ix < loop_ub; ix++) {
                b_A[ix] = x[ix];
              }

              b_A[qq - 1] = x[qq - 1] + 1.0;
              b_s_data[q] = -b_s_data[q];
            } else {
              b_s_data[q] = 0.0;
            }
          }

          for (jj = qp1; jj <= p; jj++) {
            qjj = q + n * (jj - 1);
            if (apply_transform) {
              nrm = 0.0;
              if (nmq + 1 >= 1) {
                ix = qq;
                iy = qjj;
                for (k = 0; k <= nmq; k++) {
                  nrm += b_A[ix - 1] * b_A[iy];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / b_A[q + b_A.size(0) * q]);
              x.set_size(b_A.size(0), b_A.size(1));
              loop_ub = b_A.size(0) * b_A.size(1);
              for (ix = 0; ix < loop_ub; ix++) {
                x[ix] = b_A[ix];
              }

              blas::b_xaxpy(nmq + 1, nrm, qq, x, qjj + 1);
              b_A.set_size(x.size(0), x.size(1));
              loop_ub = x.size(0) * x.size(1);
              for (ix = 0; ix < loop_ub; ix++) {
                b_A[ix] = x[ix];
              }
            }

            e_data[jj - 1] = b_A[qjj];
          }

          if (q + 1 <= nct) {
            for (qjj = q + 1; qjj <= n; qjj++) {
              U[(qjj + U.size(0) * q) - 1] = b_A[(qjj + b_A.size(0) * q) - 1];
            }
          }

          if (q + 1 <= y) {
            b_e_data.set((&e_data[0]), iter);
            nrm = blas::xnrm2(p - 1, b_e_data, 2);
            if (nrm == 0.0) {
              e_data[0] = 0.0;
            } else {
              if (e_data[1] < 0.0) {
                e_data[0] = -nrm;
              } else {
                e_data[0] = nrm;
              }

              b_x.set_size(iter);
              for (ix = 0; ix < iter; ix++) {
                b_x[ix] = e_data[ix];
              }

              if (std::abs(e_data[0]) >= 1.0020841800044864E-292) {
                nrm = 1.0 / e_data[0];
                for (k = qp1; k <= p; k++) {
                  b_x[k - 1] = nrm * b_x[k - 1];
                }
              } else {
                for (k = qp1; k <= p; k++) {
                  b_x[k - 1] = b_x[k - 1] / e_data[0];
                }
              }

              iter = b_x.size(0);
              loop_ub = b_x.size(0);
              for (ix = 0; ix < loop_ub; ix++) {
                e_data[ix] = b_x[ix];
              }

              e_data[1] = b_x[1] + 1.0;
              e_data[0] = -e_data[0];
              for (qjj = qp1; qjj <= n; qjj++) {
                work[qjj - 1] = 0.0;
              }

              for (jj = qp1; jj <= p; jj++) {
                blas::xaxpy(nmq, e_data[jj - 1], b_A, n * (jj - 1) + 2, work, 2);
              }

              for (jj = qp1; jj <= p; jj++) {
                x.set_size(b_A.size(0), b_A.size(1));
                loop_ub = b_A.size(0) * b_A.size(1);
                for (ix = 0; ix < loop_ub; ix++) {
                  x[ix] = b_A[ix];
                }

                blas::xaxpy(nmq, -e_data[jj - 1] / e_data[1], work, 2, x, n *
                            (jj - 1) + 2);
                b_A.set_size(x.size(0), x.size(1));
                loop_ub = x.size(0) * x.size(1);
                for (ix = 0; ix < loop_ub; ix++) {
                  b_A[ix] = x[ix];
                }
              }
            }

            if (qp1 <= p) {
              std::memcpy(&Vf_data[qp1 + -1], &e_data[qp1 + -1], ((p - qp1) + 1)
                          * sizeof(double));
            }
          }
        }

        m = A.size(1) - 2;
        if (nct < A.size(1)) {
          b_s_data[nct] = b_A[nct + b_A.size(0) * nct];
        }

        if (y + 1 < A.size(1)) {
          e_data[y] = b_A[y + b_A.size(0) * (A.size(1) - 1)];
        }

        e_data[A.size(1) - 1] = 0.0;
        if (nct + 1 <= A.size(1)) {
          for (jj = nctp1; jj <= p; jj++) {
            for (qjj = 0; qjj < n; qjj++) {
              U[qjj + U.size(0) * (jj - 1)] = 0.0;
            }

            U[(jj + U.size(0) * (jj - 1)) - 1] = 1.0;
          }
        }

        for (q = nct; q >= 1; q--) {
          qp1 = q + 1;
          nmq = n - q;
          qq = (q + n * (q - 1)) - 1;
          if (b_s_data[q - 1] != 0.0) {
            for (jj = qp1; jj <= p; jj++) {
              qjj = q + n * (jj - 1);
              nrm = 0.0;
              if (nmq + 1 >= 1) {
                ix = qq;
                iy = qjj;
                for (k = 0; k <= nmq; k++) {
                  nrm += U[ix] * U[iy - 1];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / U[qq]);
              blas::b_xaxpy(nmq + 1, nrm, qq + 1, U, qjj);
            }

            for (qjj = q; qjj <= n; qjj++) {
              U[(qjj + U.size(0) * (q - 1)) - 1] = -U[(qjj + U.size(0) * (q - 1))
                - 1];
            }

            U[qq] = U[qq] + 1.0;
            for (qjj = 0; qjj <= q - 2; qjj++) {
              U[qjj + U.size(0) * (q - 1)] = 0.0;
            }
          } else {
            for (qjj = 0; qjj < n; qjj++) {
              U[qjj + U.size(0) * (q - 1)] = 0.0;
            }

            U[qq] = 1.0;
          }
        }

        for (q = p; q >= 1; q--) {
          if ((q <= y) && (e_data[0] != 0.0)) {
            for (jj = 2; jj <= p; jj++) {
              nctp1 = p * (jj - 1);
              nrm = 0.0;
              if (p - 1 >= 1) {
                ix = 1;
                iy = nctp1;
                for (k = 0; k <= p - 2; k++) {
                  nrm += Vf_data[ix] * Vf_data[iy + 1];
                  ix++;
                  iy++;
                }
              }

              nrm = -(nrm / Vf_data[1]);
              x.set_size(Vf_size_idx_0, Vf_size_idx_1);
              loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
              for (i = 0; i < loop_ub; i++) {
                x[i] = Vf_data[i];
              }

              blas::b_xaxpy(p - 1, nrm, 2, x, nctp1 + 2);
              Vf_size_idx_0 = x.size(0);
              Vf_size_idx_1 = x.size(1);
              loop_ub = x.size(0) * x.size(1);
              for (i = 0; i < loop_ub; i++) {
                Vf_data[i] = x[i];
              }
            }
          }

          for (qjj = 0; qjj < p; qjj++) {
            Vf_data[qjj + Vf_size_idx_0 * (q - 1)] = 0.0;
          }

          Vf_data[(q + Vf_size_idx_0 * (q - 1)) - 1] = 1.0;
        }

        iter = 0;
        snorm = 0.0;
        for (q = 0; q < p; q++) {
          nrm = b_s_data[q];
          if (nrm != 0.0) {
            rt = std::abs(nrm);
            nrm /= rt;
            b_s_data[q] = rt;
            if (q + 1 < p) {
              e_data[q] /= nrm;
            }

            nctp1 = n * q;
            i = nctp1 + n;
            for (k = nctp1 + 1; k <= i; k++) {
              U[k - 1] = nrm * U[k - 1];
            }
          }

          if ((q + 1 < p) && (e_data[q] != 0.0)) {
            rt = std::abs(e_data[q]);
            nrm = rt / e_data[q];
            e_data[q] = rt;
            b_s_data[q + 1] *= nrm;
            nctp1 = p * (q + 1);
            x.set_size(Vf_size_idx_0, Vf_size_idx_1);
            loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
            for (i = 0; i < loop_ub; i++) {
              x[i] = Vf_data[i];
            }

            i = nctp1 + p;
            for (k = nctp1 + 1; k <= i; k++) {
              x[k - 1] = nrm * x[k - 1];
            }

            Vf_size_idx_0 = x.size(0);
            Vf_size_idx_1 = x.size(1);
            loop_ub = x.size(0) * x.size(1);
            for (i = 0; i < loop_ub; i++) {
              Vf_data[i] = x[i];
            }
          }

          nrm = std::abs(b_s_data[q]);
          rt = std::abs(e_data[q]);
          if ((nrm > rt) || rtIsNaN(rt)) {
            rt = nrm;
          }

          if ((!(snorm > rt)) && (!rtIsNaN(rt))) {
            snorm = rt;
          }
        }

        while ((m + 2 > 0) && (iter < 75)) {
          qjj = m;
          int exitg1;
          do {
            exitg1 = 0;
            q = qjj + 1;
            if (qjj + 1 == 0) {
              exitg1 = 1;
            } else {
              nrm = std::abs(e_data[qjj]);
              if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s_data[qjj]) +
                    std::abs(b_s_data[qjj + 1]))) || (nrm <=
                   1.0020841800044864E-292) || ((iter > 20) && (nrm <=
                    2.2204460492503131E-16 * snorm))) {
                e_data[qjj] = 0.0;
                exitg1 = 1;
              } else {
                qjj--;
              }
            }
          } while (exitg1 == 0);

          if (qjj + 1 == m + 1) {
            nctp1 = 4;
          } else {
            boolean_T exitg2;
            nmq = m + 2;
            nctp1 = m + 2;
            exitg2 = false;
            while ((!exitg2) && (nctp1 >= qjj + 1)) {
              nmq = nctp1;
              if (nctp1 == qjj + 1) {
                exitg2 = true;
              } else {
                nrm = 0.0;
                if (nctp1 < m + 2) {
                  nrm = std::abs(e_data[nctp1 - 1]);
                }

                if (nctp1 > qjj + 2) {
                  nrm += std::abs(e_data[nctp1 - 2]);
                }

                rt = std::abs(b_s_data[nctp1 - 1]);
                if ((rt <= 2.2204460492503131E-16 * nrm) || (rt <=
                     1.0020841800044864E-292)) {
                  b_s_data[nctp1 - 1] = 0.0;
                  exitg2 = true;
                } else {
                  nctp1--;
                }
              }
            }

            if (nmq == qjj + 1) {
              nctp1 = 3;
            } else if (nmq == m + 2) {
              nctp1 = 1;
            } else {
              nctp1 = 2;
              q = nmq;
            }
          }

          switch (nctp1) {
           case 1:
            {
              rt = e_data[m];
              e_data[m] = 0.0;
              i = m + 1;
              for (k = i; k >= q + 1; k--) {
                blas::xrotg(&b_s_data[k - 1], &rt, &sqds, &scale);
                if (k > q + 1) {
                  rt = -scale * e_data[0];
                  e_data[0] *= sqds;
                }

                x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                for (ix = 0; ix < loop_ub; ix++) {
                  x[ix] = Vf_data[ix];
                }

                if (p >= 1) {
                  ix = p * (k - 1);
                  iy = p * (m + 1);
                  for (nctp1 = 0; nctp1 < p; nctp1++) {
                    double temp;
                    temp = sqds * x[ix] + scale * x[iy];
                    x[iy] = sqds * x[iy] - scale * x[ix];
                    x[ix] = temp;
                    iy++;
                    ix++;
                  }
                }

                Vf_size_idx_0 = x.size(0);
                Vf_size_idx_1 = x.size(1);
                loop_ub = x.size(0) * x.size(1);
                for (ix = 0; ix < loop_ub; ix++) {
                  Vf_data[ix] = x[ix];
                }
              }
            }
            break;

           case 2:
            {
              rt = e_data[q - 1];
              e_data[q - 1] = 0.0;
              for (k = q + 1; k <= m + 2; k++) {
                double b;
                blas::xrotg(&b_s_data[k - 1], &rt, &sqds, &scale);
                b = e_data[k - 1];
                rt = -scale * b;
                e_data[k - 1] = b * sqds;
                if (n >= 1) {
                  ix = n * (k - 1);
                  iy = n * (q - 1);
                  for (nctp1 = 0; nctp1 < n; nctp1++) {
                    double temp;
                    temp = sqds * U[ix] + scale * U[iy];
                    U[iy] = sqds * U[iy] - scale * U[ix];
                    U[ix] = temp;
                    iy++;
                    ix++;
                  }
                }
              }
            }
            break;

           case 3:
            {
              double b;
              double temp;
              nmq = m + 1;
              nrm = b_s_data[m + 1];
              scale = std::abs(nrm);
              rt = std::abs(b_s_data[m]);
              if ((!(scale > rt)) && (!rtIsNaN(rt))) {
                scale = rt;
              }

              rt = std::abs(e_data[m]);
              if ((!(scale > rt)) && (!rtIsNaN(rt))) {
                scale = rt;
              }

              rt = std::abs(b_s_data[q]);
              if ((!(scale > rt)) && (!rtIsNaN(rt))) {
                scale = rt;
              }

              rt = std::abs(e_data[q]);
              if ((!(scale > rt)) && (!rtIsNaN(rt))) {
                scale = rt;
              }

              temp = nrm / scale;
              nrm = b_s_data[m] / scale;
              rt = e_data[m] / scale;
              sqds = b_s_data[q] / scale;
              b = ((nrm + temp) * (nrm - temp) + rt * rt) / 2.0;
              nrm = temp * rt;
              nrm *= nrm;
              if ((b != 0.0) || (nrm != 0.0)) {
                rt = std::sqrt(b * b + nrm);
                if (b < 0.0) {
                  rt = -rt;
                }

                rt = nrm / (b + rt);
              } else {
                rt = 0.0;
              }

              rt += (sqds + temp) * (sqds - temp);
              nrm = sqds * (e_data[q] / scale);
              for (k = q + 1; k <= nmq; k++) {
                blas::xrotg(&rt, &nrm, &sqds, &scale);
                if (k > q + 1) {
                  e_data[0] = rt;
                }

                b = e_data[k - 1];
                nrm = b_s_data[k - 1];
                e_data[k - 1] = sqds * b - scale * nrm;
                rt = scale * b_s_data[k];
                b_s_data[k] *= sqds;
                x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                for (i = 0; i < loop_ub; i++) {
                  x[i] = Vf_data[i];
                }

                if (p >= 1) {
                  ix = p * (k - 1);
                  iy = p * k;
                  for (nctp1 = 0; nctp1 < p; nctp1++) {
                    temp = sqds * x[ix] + scale * x[iy];
                    x[iy] = sqds * x[iy] - scale * x[ix];
                    x[ix] = temp;
                    iy++;
                    ix++;
                  }
                }

                Vf_size_idx_0 = x.size(0);
                Vf_size_idx_1 = x.size(1);
                loop_ub = x.size(0) * x.size(1);
                for (i = 0; i < loop_ub; i++) {
                  Vf_data[i] = x[i];
                }

                b_s_data[k - 1] = sqds * nrm + scale * b;
                blas::xrotg(&b_s_data[k - 1], &rt, &sqds, &scale);
                rt = sqds * e_data[k - 1] + scale * b_s_data[k];
                b_s_data[k] = -scale * e_data[k - 1] + sqds * b_s_data[k];
                nrm = scale * e_data[k];
                e_data[k] *= sqds;
                if (n >= 1) {
                  ix = n * (k - 1);
                  iy = n * k;
                  for (nctp1 = 0; nctp1 < n; nctp1++) {
                    temp = sqds * U[ix] + scale * U[iy];
                    U[iy] = sqds * U[iy] - scale * U[ix];
                    U[ix] = temp;
                    iy++;
                    ix++;
                  }
                }
              }

              e_data[m] = rt;
              iter++;
            }
            break;

           default:
            {
              if (b_s_data[q] < 0.0) {
                b_s_data[q] = -b_s_data[q];
                nctp1 = p * q;
                x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                for (i = 0; i < loop_ub; i++) {
                  x[i] = Vf_data[i];
                }

                i = nctp1 + p;
                for (k = nctp1 + 1; k <= i; k++) {
                  x[k - 1] = -x[k - 1];
                }

                Vf_size_idx_0 = x.size(0);
                Vf_size_idx_1 = x.size(1);
                loop_ub = x.size(0) * x.size(1);
                for (i = 0; i < loop_ub; i++) {
                  Vf_data[i] = x[i];
                }
              }

              qp1 = q + 1;
              while ((q + 1 < p) && (b_s_data[q] < b_s_data[qp1])) {
                double temp;
                rt = b_s_data[q];
                b_s_data[q] = b_s_data[qp1];
                b_s_data[qp1] = rt;
                if (q + 1 < p) {
                  ix = p * q;
                  iy = p * (q + 1);
                  x.set_size(Vf_size_idx_0, Vf_size_idx_1);
                  loop_ub = Vf_size_idx_0 * Vf_size_idx_1;
                  for (i = 0; i < loop_ub; i++) {
                    x[i] = Vf_data[i];
                  }

                  for (k = 0; k < p; k++) {
                    temp = x[ix];
                    x[ix] = x[iy];
                    x[iy] = temp;
                    ix++;
                    iy++;
                  }

                  Vf_size_idx_0 = x.size(0);
                  Vf_size_idx_1 = x.size(1);
                  loop_ub = x.size(0) * x.size(1);
                  for (i = 0; i < loop_ub; i++) {
                    Vf_data[i] = x[i];
                  }
                }

                ix = n * q;
                iy = n * (q + 1);
                for (k = 0; k < n; k++) {
                  temp = U[ix];
                  U[ix] = U[iy];
                  U[iy] = temp;
                  ix++;
                  iy++;
                }

                q = qp1;
                qp1++;
              }

              iter = 0;
              m--;
            }
            break;
          }
        }
      }

      s_size[0] = A.size(1);
      V_size[0] = A.size(1);
      V_size[1] = A.size(1);
      for (k = 0; k < p; k++) {
        s_data[k] = b_s_data[k];
        for (nctp1 = 0; nctp1 < p; nctp1++) {
          V_data[nctp1 + V_size[0] * k] = Vf_data[nctp1 + Vf_size_idx_0 * k];
        }
      }
    }
  }
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
