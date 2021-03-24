//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgeqp3.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "coder_array.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Declarations
static double rt_hypotd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else {
    if (!rtIsNaN(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

//
// Arguments    : double A_data[]
//                int A_size[2]
//                double tau_data[]
//                int tau_size[1]
//                int jpvt_data[]
//                int jpvt_size[2]
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    namespace lapack
    {
      void xgeqp3(double A_data[], int A_size[2], double tau_data[], int
                  tau_size[1], int jpvt_data[], int jpvt_size[2])
      {
        array<double, 2U> b_A_data;
        array<double, 2U> x;
        double vn1_data[3];
        double vn2_data[3];
        double work_data[3];
        double s;
        double smax;
        double temp2;
        int itemp;
        int jA;
        int m;
        int n;
        int u1;
        m = A_size[0];
        n = A_size[1];
        itemp = A_size[0];
        u1 = A_size[1];
        if (itemp < u1) {
          u1 = itemp;
        }

        tau_size[0] = u1;
        if (0 <= u1 - 1) {
          std::memset(&tau_data[0], 0, u1 * sizeof(double));
        }

        if ((A_size[0] == 0) || (A_size[1] == 0) || (u1 < 1)) {
          int lastc;
          jpvt_size[0] = 1;
          jpvt_size[1] = A_size[1];
          lastc = A_size[1];
          if (0 <= lastc - 1) {
            std::memset(&jpvt_data[0], 0, lastc * sizeof(int));
          }

          for (int pvt = 0; pvt < n; pvt++) {
            jpvt_data[pvt] = pvt + 1;
          }
        } else {
          int lastc;
          int lastv;
          int ma;
          int pvt;
          jpvt_size[0] = 1;
          jpvt_size[1] = A_size[1];
          lastc = A_size[1];
          if (0 <= lastc - 1) {
            std::memset(&jpvt_data[0], 0, lastc * sizeof(int));
          }

          for (lastv = 0; lastv < n; lastv++) {
            jpvt_data[lastv] = lastv + 1;
          }

          ma = A_size[0];
          lastc = A_size[1];
          if (0 <= lastc - 1) {
            std::memset(&work_data[0], 0, lastc * sizeof(double));
          }

          lastc = A_size[1];
          if (0 <= lastc - 1) {
            std::memset(&vn1_data[0], 0, lastc * sizeof(double));
          }

          lastc = A_size[1];
          if (0 <= lastc - 1) {
            std::memset(&vn2_data[0], 0, lastc * sizeof(double));
          }

          for (pvt = 0; pvt < n; pvt++) {
            smax = blas::xnrm2(m, A_data, pvt * ma + 1);
            vn1_data[pvt] = smax;
            vn2_data[pvt] = smax;
          }

          for (int i = 0; i < u1; i++) {
            int b_i;
            int i1;
            int ii;
            int ip1;
            int ix;
            int iy;
            int knt;
            int mmi;
            int nmi;
            ip1 = i + 2;
            iy = i * ma;
            ii = iy + i;
            nmi = n - i;
            mmi = m - i;
            if (nmi < 1) {
              itemp = -1;
            } else {
              itemp = 0;
              if (nmi > 1) {
                ix = i;
                smax = std::abs(vn1_data[i]);
                for (lastv = 2; lastv <= nmi; lastv++) {
                  ix++;
                  s = std::abs(vn1_data[ix]);
                  if (s > smax) {
                    itemp = lastv - 1;
                    smax = s;
                  }
                }
              }
            }

            pvt = i + itemp;
            if (pvt + 1 != i + 1) {
              ix = pvt * ma;
              x.set_size(A_size[0], A_size[1]);
              lastc = A_size[0] * A_size[1];
              for (b_i = 0; b_i < lastc; b_i++) {
                x[b_i] = A_data[b_i];
              }

              for (lastv = 0; lastv < m; lastv++) {
                smax = x[ix];
                x[ix] = x[iy];
                x[iy] = smax;
                ix++;
                iy++;
              }

              A_size[0] = x.size(0);
              A_size[1] = x.size(1);
              lastc = x.size(1);
              for (b_i = 0; b_i < lastc; b_i++) {
                itemp = x.size(0);
                for (i1 = 0; i1 < itemp; i1++) {
                  A_data[i1 + A_size[0] * b_i] = x[i1 + x.size(0) * b_i];
                }
              }

              itemp = static_cast<signed char>(jpvt_data[pvt]);
              jpvt_data[pvt] = static_cast<signed char>(jpvt_data[i]);
              jpvt_data[i] = itemp;
              vn1_data[pvt] = vn1_data[i];
              vn2_data[pvt] = vn2_data[i];
            }

            if (i + 1 < m) {
              temp2 = A_data[ii];
              pvt = ii + 2;
              tau_data[i] = 0.0;
              if (mmi > 0) {
                smax = blas::xnrm2(mmi - 1, A_data, ii + 2);
                if (smax != 0.0) {
                  s = rt_hypotd_snf(A_data[ii], smax);
                  if (A_data[ii] >= 0.0) {
                    s = -s;
                  }

                  if (std::abs(s) < 1.0020841800044864E-292) {
                    knt = -1;
                    b_i = ii + mmi;
                    do {
                      knt++;
                      x.set_size(A_size[0], A_size[1]);
                      lastc = A_size[0] * A_size[1];
                      for (i1 = 0; i1 < lastc; i1++) {
                        x[i1] = A_data[i1];
                      }

                      for (lastv = pvt; lastv <= b_i; lastv++) {
                        x[lastv - 1] = 9.9792015476736E+291 * x[lastv - 1];
                      }

                      A_size[0] = x.size(0);
                      A_size[1] = x.size(1);
                      lastc = x.size(1);
                      for (i1 = 0; i1 < lastc; i1++) {
                        itemp = x.size(0);
                        for (lastv = 0; lastv < itemp; lastv++) {
                          A_data[lastv + A_size[0] * i1] = x[lastv + x.size(0) *
                            i1];
                        }
                      }

                      s *= 9.9792015476736E+291;
                      temp2 *= 9.9792015476736E+291;
                    } while (!(std::abs(s) >= 1.0020841800044864E-292));

                    s = rt_hypotd_snf(temp2, blas::xnrm2(mmi - 1, (double *)
                      x.data(), ii + 2));
                    if (temp2 >= 0.0) {
                      s = -s;
                    }

                    tau_data[i] = (s - temp2) / s;
                    smax = 1.0 / (temp2 - s);
                    for (lastv = pvt; lastv <= b_i; lastv++) {
                      x[lastv - 1] = smax * x[lastv - 1];
                    }

                    A_size[0] = x.size(0);
                    A_size[1] = x.size(1);
                    lastc = x.size(1);
                    for (b_i = 0; b_i < lastc; b_i++) {
                      itemp = x.size(0);
                      for (i1 = 0; i1 < itemp; i1++) {
                        A_data[i1 + A_size[0] * b_i] = x[i1 + x.size(0) * b_i];
                      }
                    }

                    for (lastv = 0; lastv <= knt; lastv++) {
                      s *= 1.0020841800044864E-292;
                    }

                    temp2 = s;
                  } else {
                    tau_data[i] = (s - A_data[ii]) / s;
                    smax = 1.0 / (A_data[ii] - s);
                    x.set_size(A_size[0], A_size[1]);
                    lastc = A_size[0] * A_size[1];
                    for (b_i = 0; b_i < lastc; b_i++) {
                      x[b_i] = A_data[b_i];
                    }

                    b_i = ii + mmi;
                    for (lastv = pvt; lastv <= b_i; lastv++) {
                      x[lastv - 1] = smax * x[lastv - 1];
                    }

                    A_size[0] = x.size(0);
                    A_size[1] = x.size(1);
                    lastc = x.size(1);
                    for (b_i = 0; b_i < lastc; b_i++) {
                      itemp = x.size(0);
                      for (i1 = 0; i1 < itemp; i1++) {
                        A_data[i1 + A_size[0] * b_i] = x[i1 + x.size(0) * b_i];
                      }
                    }

                    temp2 = s;
                  }
                }
              }

              A_data[ii] = temp2;
            } else {
              tau_data[i] = 0.0;
            }

            if (i + 1 < n) {
              temp2 = A_data[ii];
              A_data[ii] = 1.0;
              jA = (ii + ma) + 1;
              if (tau_data[i] != 0.0) {
                boolean_T exitg2;
                lastv = mmi - 1;
                itemp = (ii + mmi) - 1;
                while ((lastv + 1 > 0) && (A_data[itemp] == 0.0)) {
                  lastv--;
                  itemp--;
                }

                lastc = nmi - 2;
                exitg2 = false;
                while ((!exitg2) && (lastc + 1 > 0)) {
                  int exitg1;
                  itemp = jA + lastc * ma;
                  knt = itemp;
                  do {
                    exitg1 = 0;
                    if (knt <= itemp + lastv) {
                      if (A_data[knt - 1] != 0.0) {
                        exitg1 = 1;
                      } else {
                        knt++;
                      }
                    } else {
                      lastc--;
                      exitg1 = 2;
                    }
                  } while (exitg1 == 0);

                  if (exitg1 == 1) {
                    exitg2 = true;
                  }
                }
              } else {
                lastv = -1;
                lastc = -1;
              }

              if (lastv + 1 > 0) {
                if (lastc + 1 != 0) {
                  if (0 <= lastc) {
                    std::memset(&work_data[0], 0, (lastc + 1) * sizeof(double));
                  }

                  iy = 0;
                  b_i = jA + ma * lastc;
                  for (itemp = jA; ma < 0 ? itemp >= b_i : itemp <= b_i; itemp +=
                       ma) {
                    ix = ii;
                    smax = 0.0;
                    i1 = itemp + lastv;
                    for (knt = itemp; knt <= i1; knt++) {
                      smax += A_data[knt - 1] * A_data[ix];
                      ix++;
                    }

                    work_data[iy] += smax;
                    iy++;
                  }
                }

                if (!(-tau_data[i] == 0.0)) {
                  itemp = 0;
                  for (pvt = 0; pvt <= lastc; pvt++) {
                    if (work_data[itemp] != 0.0) {
                      smax = work_data[itemp] * -tau_data[i];
                      ix = ii;
                      b_i = lastv + jA;
                      for (knt = jA; knt <= b_i; knt++) {
                        A_data[knt - 1] += A_data[ix] * smax;
                        ix++;
                      }
                    }

                    itemp++;
                    jA += ma;
                  }
                }
              }

              A_data[ii] = temp2;
            }

            for (pvt = ip1; pvt <= n; pvt++) {
              itemp = i + (pvt - 1) * ma;
              smax = vn1_data[pvt - 1];
              if (smax != 0.0) {
                s = std::abs(A_data[itemp]) / smax;
                s = 1.0 - s * s;
                if (s < 0.0) {
                  s = 0.0;
                }

                temp2 = smax / vn2_data[pvt - 1];
                temp2 = s * (temp2 * temp2);
                if (temp2 <= 1.4901161193847656E-8) {
                  if (i + 1 < m) {
                    b_A_data.set((&A_data[0]), A_size[0], A_size[1]);
                    smax = blas::b_xnrm2(mmi - 1, b_A_data, itemp + 2);
                    vn1_data[pvt - 1] = smax;
                    vn2_data[pvt - 1] = smax;
                  } else {
                    vn1_data[pvt - 1] = 0.0;
                    vn2_data[pvt - 1] = 0.0;
                  }
                } else {
                  vn1_data[pvt - 1] = smax * std::sqrt(s);
                }
              }
            }
          }
        }
      }
    }
  }
}

//
// File trailer for xgeqp3.cpp
//
// [EOF]
//
