//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ecLQR_laine.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "ecLQR_laine.h"
#include "ecLQR_laine_data.h"
#include "ecLQR_laine_initialize.h"
#include "ecLQR_laine_types.h"
#include "mldivide.h"
#include "mtimes.h"
#include "pinv.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
//
// ECLQR_SIDERIS solve LQR with equality constraint using sideris method
//               in paper Efficient computation of feedback control for equality-constrained lqr
//  It uses the general input output structure:
//  Inputs:
//   -- param   a struct contains all necessary parameters
//   -- xN      The final target that state must reach at N
//   -- A_list B_list lists of system dynamics
//   -- C_list, D_list, G_list, r_list, h_list lists of constraints
//  Outputs:
//   -- Soln
// Arguments    : const struct0_T *param
//                const double xN[3]
//                const double A_list[900]
//                const double B_list[900]
//                const double C_list[900]
//                const double D_list[900]
//                const double G_list[900]
//                const double r_list[300]
//                const double h_list[300]
//                coder::array<double, 2U> &x_list
//                coder::array<double, 3U> &K_list
//                coder::array<double, 2U> &k_list
// Return Type  : void
//
void ecLQR_laine(const struct0_T *param, const double xN[3], const double
                 A_list[900], const double B_list[900], const double C_list[900],
                 const double D_list[900], const double G_list[900], const
                 double r_list[300], const double h_list[300], coder::array<
                 double, 2U> &x_list, coder::array<double, 3U> &K_list, coder::
                 array<double, 2U> &k_list)
{
  coder::array<double, 3U> b_D_list;
  coder::array<double, 2U> C_data;
  coder::array<double, 2U> Hxt2;
  coder::array<double, 2U> Nut;
  coder::array<double, 2U> Nxt;
  coder::array<double, 2U> S;
  coder::array<double, 2U> U;
  coder::array<double, 2U> b;
  coder::array<double, 2U> b_V_data;
  coder::array<double, 2U> b_b;
  coder::array<double, 2U> c_V_data;
  coder::array<double, 2U> e_y_tmp;
  coder::array<double, 2U> r;
  coder::array<double, 2U> unusedU1;
  coder::array<double, 1U> C;
  coder::array<double, 1U> hlt2;
  coder::array<double, 1U> nlt;
  double b_C_list[1800];
  double r_list_data[600];
  double K[9];
  double Muut[9];
  double Muut_tmp[9];
  double Muxt[9];
  double V[9];
  double V_data[9];
  double Vxxt[9];
  double b_C_data[9];
  double b_y_tmp[9];
  double c_y_tmp[9];
  double d_y_tmp[9];
  double y_tmp[9];
  double b_C[3];
  double k[3];
  double mult[3];
  double vxlt[3];
  double d;
  double d1;
  double d2;
  double d3;
  double nu;
  int C_size[2];
  int V_size[2];
  int C_list_tmp;
  int aoffset;
  int b_i;
  int b_loop_ub;
  int boffset;
  int d_i;
  int i;
  int i1;
  int inner;
  int input_sizes_idx_0;
  int j;
  int loop_ub;
  int mc;
  int r_list_size_idx_0;
  signed char b_input_sizes_idx_0;
  boolean_T empty_non_axis_sizes;
  if (!isInitialized_ecLQR_laine) {
    ecLQR_laine_initialize();
  }

  coder::tic();

  // 1. necessary variables
  nu = param->nu;

  // lt
  // lT
  std::memcpy(&Vxxt[0], &param->Qf[0], 9U * sizeof(double));

  //  convert Gx=h into Cx+Du=r format for Laine
  if (1.0 > param->N) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(param->N);
  }

  for (i = 0; i < 100; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      C_list_tmp = 3 * i1 + 9 * i;
      input_sizes_idx_0 = 6 * i1 + 18 * i;
      b_C_list[input_sizes_idx_0] = C_list[C_list_tmp];
      b_C_list[input_sizes_idx_0 + 1] = C_list[C_list_tmp + 1];
      b_C_list[input_sizes_idx_0 + 2] = C_list[C_list_tmp + 2];
    }
  }

  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      C_list_tmp = 3 * i1 + 9 * i;
      input_sizes_idx_0 = 6 * i1 + 18 * i;
      b_C_list[input_sizes_idx_0 + 3] = G_list[C_list_tmp];
      b_C_list[input_sizes_idx_0 + 4] = G_list[C_list_tmp + 1];
      b_C_list[input_sizes_idx_0 + 5] = G_list[C_list_tmp + 2];
    }
  }

  if (static_cast<int>(param->ncx) != 0) {
    empty_non_axis_sizes = false;
  } else if (static_cast<int>(param->nu) != 0) {
    empty_non_axis_sizes = false;
  } else {
    empty_non_axis_sizes = (static_cast<int>(param->N) == 1);
  }

  if (!empty_non_axis_sizes) {
    input_sizes_idx_0 = static_cast<int>(param->ncx);
  } else {
    input_sizes_idx_0 = 0;
  }

  b_D_list.set_size((input_sizes_idx_0 + 3), 3, 100);
  if (1.0 > param->N) {
    i = 0;
  } else {
    i = static_cast<int>(param->N);
  }

  if (i != 0) {
    b_input_sizes_idx_0 = 3;
  } else {
    b_input_sizes_idx_0 = 0;
  }

  r_list_size_idx_0 = b_input_sizes_idx_0 + 3;
  loop_ub = b_input_sizes_idx_0;
  for (i = 0; i < 100; i++) {
    for (i1 = 0; i1 < 3; i1++) {
      C_list_tmp = 3 * i1 + 9 * i;
      b_D_list[b_D_list.size(0) * i1 + b_D_list.size(0) * 3 * i] =
        D_list[C_list_tmp];
      b_D_list[(b_D_list.size(0) * i1 + b_D_list.size(0) * 3 * i) + 1] =
        D_list[C_list_tmp + 1];
      b_D_list[(b_D_list.size(0) * i1 + b_D_list.size(0) * 3 * i) + 2] =
        D_list[C_list_tmp + 2];
      for (j = 0; j < input_sizes_idx_0; j++) {
        b_D_list[((j + b_D_list.size(0) * i1) + b_D_list.size(0) * 3 * i) + 3] =
          0.0;
      }

      r_list_data[i1 + (b_input_sizes_idx_0 + 3) * i] = r_list[i1 + 3 * i];
    }

    for (i1 = 0; i1 < loop_ub; i1++) {
      r_list_data[(i1 + (b_input_sizes_idx_0 + 3) * i) + 3] = h_list[i1 + 3 * i];
    }
  }

  //  2. init cost to go and constraint to go
  //  use notations in paper
  //  cost to go is 0.5*x'*VxxT*x + vxlT*x
  //  constraint to go is HxT and hlT
  //  2.1 equation 9
  for (i = 0; i < 9; i++) {
    Muut[i] = -param->Qf[i];
  }

  //  cost to go and costraint to go during iteration
  Hxt2.set_size(3, 3);
  hlt2.set_size(3);
  d = xN[0];
  d1 = xN[1];
  d2 = xN[2];
  for (i = 0; i < 3; i++) {
    C_list_tmp = 3 * i + 9 * (static_cast<int>(param->N) - 1);
    Hxt2[Hxt2.size(0) * i] = G_list[C_list_tmp];
    Hxt2[Hxt2.size(0) * i + 1] = G_list[C_list_tmp + 1];
    Hxt2[Hxt2.size(0) * i + 2] = G_list[C_list_tmp + 2];
    vxlt[i] = (Muut[i] * d + Muut[i + 3] * d1) + Muut[i + 6] * d2;
    hlt2[i] = h_list[i + 3 * (static_cast<int>(param->N) - 1)];
  }

  k_list.set_size((static_cast<int>(param->nu)), (static_cast<int>(param->N)));

  //  from 0 to N-1
  K_list.set_size((static_cast<int>(param->nu)), (static_cast<int>(param->nx)),
                  (static_cast<int>(param->N)));

  //  from 0 to N-1
  i = static_cast<int>(((-1.0 - param->N) + 1.0) / -1.0);
  if (0 <= i - 1) {
    b_loop_ub = b_D_list.size(0);
  }

  for (b_i = 0; b_i < i; b_i++) {
    double c_i;
    double d4;
    double d5;
    int S_tmp;
    int n;
    signed char i2;
    c_i = param->N + -static_cast<double>(b_i);

    //  necessary varialbe conversion
    //  equation 12
    for (i1 = 0; i1 < 3; i1++) {
      input_sizes_idx_0 = i1 + 9 * (static_cast<int>(c_i) - 1);
      y_tmp[3 * i1] = A_list[input_sizes_idx_0];
      b_y_tmp[3 * i1] = B_list[input_sizes_idx_0];
      C_list_tmp = 3 * i1 + 1;
      y_tmp[C_list_tmp] = A_list[input_sizes_idx_0 + 3];
      b_y_tmp[C_list_tmp] = B_list[input_sizes_idx_0 + 3];
      C_list_tmp = 3 * i1 + 2;
      y_tmp[C_list_tmp] = A_list[input_sizes_idx_0 + 6];
      b_y_tmp[C_list_tmp] = B_list[input_sizes_idx_0 + 6];
    }

    S_tmp = 9 * (static_cast<int>(c_i) - 1);
    coder::internal::blas::mtimes(Hxt2, *(double (*)[9])&A_list[S_tmp], S);
    Nxt.set_size((S.size(0) + 6), 3);
    loop_ub = S.size(0);
    for (i1 = 0; i1 < 3; i1++) {
      d = 0.0;
      for (j = 0; j < 3; j++) {
        aoffset = i1 + 3 * j;
        d += b_y_tmp[aoffset] * vxlt[j];
        Muut_tmp[aoffset] = (b_y_tmp[i1] * Vxxt[3 * j] + b_y_tmp[i1 + 3] * Vxxt
                             [3 * j + 1]) + b_y_tmp[i1 + 6] * Vxxt[3 * j + 2];
      }

      mult[i1] = d;
      d = Muut_tmp[i1];
      d1 = Muut_tmp[i1 + 3];
      d2 = Muut_tmp[i1 + 6];
      for (j = 0; j < 3; j++) {
        aoffset = 3 * j + S_tmp;
        C_list_tmp = i1 + 3 * j;
        Muut[C_list_tmp] = param->R[C_list_tmp] + ((d * B_list[aoffset] + d1 *
          B_list[aoffset + 1]) + d2 * B_list[aoffset + 2]);
        Muxt[C_list_tmp] = (d * A_list[aoffset] + d1 * A_list[aoffset + 1]) + d2
          * A_list[aoffset + 2];
      }

      for (j = 0; j < 6; j++) {
        Nxt[j + Nxt.size(0) * i1] = b_C_list[(j + 6 * i1) + 18 * (static_cast<
          int>(c_i) - 1)];
      }

      for (j = 0; j < loop_ub; j++) {
        Nxt[(j + Nxt.size(0) * i1) + 6] = S[j + S.size(0) * i1];
      }
    }

    coder::internal::blas::mtimes(Hxt2, *(double (*)[9])&B_list[S_tmp], S);
    Nut.set_size((b_D_list.size(0) + S.size(0)), 3);
    loop_ub = S.size(0);
    for (i1 = 0; i1 < 3; i1++) {
      for (j = 0; j < b_loop_ub; j++) {
        Nut[j + Nut.size(0) * i1] = b_D_list[(j + b_D_list.size(0) * i1) +
          b_D_list.size(0) * 3 * (static_cast<int>(c_i) - 1)];
      }

      for (j = 0; j < loop_ub; j++) {
        Nut[(j + b_loop_ub) + Nut.size(0) * i1] = S[j + S.size(0) * i1];
      }
    }

    C_list_tmp = Hxt2.size(0) - 1;
    input_sizes_idx_0 = Hxt2.size(1);
    C.set_size(Hxt2.size(0));
    for (d_i = 0; d_i <= C_list_tmp; d_i++) {
      C[d_i] = 0.0;
    }

    for (boffset = 0; boffset < input_sizes_idx_0; boffset++) {
      aoffset = boffset * Hxt2.size(0);
      for (d_i = 0; d_i <= C_list_tmp; d_i++) {
        C[d_i] = C[d_i] + Hxt2[aoffset + d_i] * 0.0;
      }
    }

    nlt.set_size(((b_input_sizes_idx_0 + C.size(0)) + 3));
    for (i1 = 0; i1 < r_list_size_idx_0; i1++) {
      nlt[i1] = r_list_data[i1 + (b_input_sizes_idx_0 + 3) * (static_cast<int>
        (c_i) - 1)];
    }

    loop_ub = C.size(0);
    for (i1 = 0; i1 < loop_ub; i1++) {
      nlt[(i1 + b_input_sizes_idx_0) + 3] = C[i1] + hlt2[i1];
    }

    //  svd to get P and Z, equation 13d
    coder::svd(Nut, U, S, V);
    n = 0;
    i1 = S.size(0) * 3;
    for (boffset = 0; boffset < i1; boffset++) {
      if (S[boffset] != 0.0) {
        n++;
      }
    }

    //      % equation 14 and 15
    //      y = -pinv(Nut*P)*(Nxt*xt+nlt);
    //      w = -inv(Z'*Muut*Z)*Z'*(Muxt*xt+mult);
    if (n == 0) {
      for (i1 = 0; i1 < 3; i1++) {
        b_y_tmp[3 * i1] = V[i1];
        b_y_tmp[3 * i1 + 1] = V[i1 + 3];
        b_y_tmp[3 * i1 + 2] = V[i1 + 6];
      }

      //  equation 17 and 18
      for (i1 = 0; i1 < 3; i1++) {
        d = b_y_tmp[i1];
        d1 = b_y_tmp[i1 + 3];
        d2 = b_y_tmp[i1 + 6];
        for (j = 0; j < 3; j++) {
          d_y_tmp[i1 + 3 * j] = (d * Muut[3 * j] + d1 * Muut[3 * j + 1]) + d2 *
            Muut[3 * j + 2];
        }

        d = d_y_tmp[i1];
        d1 = d_y_tmp[i1 + 3];
        d2 = d_y_tmp[i1 + 6];
        for (j = 0; j < 3; j++) {
          c_y_tmp[i1 + 3 * j] = (d * V[3 * j] + d1 * V[3 * j + 1]) + d2 * V[3 *
            j + 2];
        }
      }

      coder::mldivide(c_y_tmp, b_y_tmp, Muut_tmp);
      for (i1 = 0; i1 < 3; i1++) {
        d = V[i1];
        d1 = V[i1 + 3];
        d2 = V[i1 + 6];
        for (j = 0; j < 3; j++) {
          c_y_tmp[i1 + 3 * j] = (d * Muut_tmp[3 * j] + d1 * Muut_tmp[3 * j + 1])
            + d2 * Muut_tmp[3 * j + 2];
        }

        d3 = c_y_tmp[i1];
        d4 = c_y_tmp[i1 + 3];
        d5 = c_y_tmp[i1 + 6];
        for (j = 0; j < 3; j++) {
          K[i1 + 3 * j] = -((d3 * Muxt[3 * j] + d4 * Muxt[3 * j + 1]) + d5 *
                            Muxt[3 * j + 2]);
        }

        d3 = 0.0;
        for (j = 0; j < 3; j++) {
          d4 = (d * Muut_tmp[3 * j] + d1 * Muut_tmp[3 * j + 1]) + d2 * Muut_tmp
            [3 * j + 2];
          c_y_tmp[i1 + 3 * j] = d4;
          d3 += d4 * mult[j];
        }

        k[i1] = -d3;
      }

      loop_ub = k_list.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        k_list[i1 + k_list.size(0) * (static_cast<int>(c_i) - 1)] = k[i1];
      }

      C_list_tmp = K_list.size(0);
      input_sizes_idx_0 = K_list.size(0);
      loop_ub = K_list.size(1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (j = 0; j < C_list_tmp; j++) {
          K_list[(j + K_list.size(0) * i1) + K_list.size(0) * K_list.size(1) * (
            static_cast<int>(c_i) - 1)] = K[j + input_sizes_idx_0 * i1];
        }
      }
    } else if (n == nu) {
      //  equation 17 and 18
      C_list_tmp = Nut.size(0);
      S.set_size(Nut.size(0), 3);
      for (j = 0; j < 3; j++) {
        input_sizes_idx_0 = j * C_list_tmp;
        boffset = j * 3;
        for (d_i = 0; d_i < C_list_tmp; d_i++) {
          S[input_sizes_idx_0 + d_i] = (Nut[d_i] * V[boffset] + Nut[Nut.size(0)
            + d_i] * V[boffset + 1]) + Nut[(Nut.size(0) << 1) + d_i] * V[boffset
            + 2];
        }
      }

      coder::pinv(S, b);
      coder::internal::blas::mtimes(V, b, Nut);
      coder::internal::blas::mtimes(Nut, Nxt, K);
      for (i1 = 0; i1 < 9; i1++) {
        K[i1] = -K[i1];
      }

      coder::pinv(S, b);
      coder::internal::blas::mtimes(V, b, Nut);
      input_sizes_idx_0 = Nut.size(1);
      k[0] = 0.0;
      k[1] = 0.0;
      k[2] = 0.0;
      for (boffset = 0; boffset < input_sizes_idx_0; boffset++) {
        aoffset = boffset * 3;
        k[0] += Nut[aoffset] * nlt[boffset];
        k[1] += Nut[aoffset + 1] * nlt[boffset];
        k[2] += Nut[aoffset + 2] * nlt[boffset];
      }

      k[0] = -k[0];
      k[1] = -k[1];
      k[2] = -k[2];
      loop_ub = k_list.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        k_list[i1 + k_list.size(0) * (static_cast<int>(c_i) - 1)] = k[i1];
      }

      C_list_tmp = K_list.size(0);
      input_sizes_idx_0 = K_list.size(0);
      loop_ub = K_list.size(1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (j = 0; j < C_list_tmp; j++) {
          K_list[(j + K_list.size(0) * i1) + K_list.size(0) * K_list.size(1) * (
            static_cast<int>(c_i) - 1)] = K[j + input_sizes_idx_0 * i1];
        }
      }
    } else {
      int m_tmp;
      if (1 > n) {
        loop_ub = 0;
      } else {
        loop_ub = n;
      }

      if (static_cast<double>(n) + 1.0 > nu) {
        i1 = 0;
        j = 0;
      } else {
        i1 = n;
        j = static_cast<int>(nu);
      }

      //  equation 17 and 18
      V_size[0] = 3;
      V_size[1] = loop_ub;
      for (aoffset = 0; aoffset < loop_ub; aoffset++) {
        V_data[3 * aoffset] = V[3 * aoffset];
        C_list_tmp = 3 * aoffset + 1;
        V_data[C_list_tmp] = V[C_list_tmp];
        C_list_tmp = 3 * aoffset + 2;
        V_data[C_list_tmp] = V[C_list_tmp];
      }

      coder::internal::blas::mtimes(Nut, V_data, V_size, e_y_tmp);
      coder::b_pinv(e_y_tmp, b_b);
      m_tmp = j - i1;
      for (j = 0; j < 3; j++) {
        input_sizes_idx_0 = j * m_tmp;
        boffset = j * 3;
        for (d_i = 0; d_i < m_tmp; d_i++) {
          aoffset = d_i * 3;
          b_C_data[input_sizes_idx_0 + d_i] = (V[aoffset % 3 + 3 * (i1 + aoffset
            / 3)] * Muut[boffset] + V[(aoffset + 1) % 3 + 3 * (i1 + (aoffset + 1)
            / 3)] * Muut[boffset + 1]) + V[(aoffset + 2) % 3 + 3 * (i1 +
            (aoffset + 2) / 3)] * Muut[boffset + 2];
        }
      }

      V_size[0] = 3;
      V_size[1] = m_tmp;
      for (j = 0; j < m_tmp; j++) {
        C_list_tmp = 3 * (i1 + j);
        V_data[3 * j] = V[C_list_tmp];
        V_data[3 * j + 1] = V[C_list_tmp + 1];
        V_data[3 * j + 2] = V[C_list_tmp + 2];
      }

      C_data.set((&b_C_data[0]), m_tmp, 3);
      coder::internal::blas::mtimes(C_data, V_data, V_size, r);
      C_size[0] = m_tmp;
      C_size[1] = 3;
      for (j = 0; j < 3; j++) {
        for (aoffset = 0; aoffset < m_tmp; aoffset++) {
          b_C_data[aoffset + m_tmp * j] = V[j + 3 * (i1 + aoffset)];
        }
      }

      coder::mldivide((double *)r.data(), r.size(), b_C_data, C_size);
      S.set_size(C_size[0], 3);
      input_sizes_idx_0 = C_size[0] * C_size[1];
      for (j = 0; j < input_sizes_idx_0; j++) {
        S[j] = b_C_data[j];
      }

      V_size[0] = 3;
      V_size[1] = loop_ub;
      for (j = 0; j < loop_ub; j++) {
        V_data[3 * j] = V[3 * j];
        C_list_tmp = 3 * j + 1;
        V_data[C_list_tmp] = V[C_list_tmp];
        C_list_tmp = 3 * j + 2;
        V_data[C_list_tmp] = V[C_list_tmp];
      }

      coder::internal::blas::mtimes(V_data, V_size, b_b, Nut);
      coder::internal::blas::mtimes(Nut, Nxt, Muut_tmp);
      for (j = 0; j < m_tmp; j++) {
        C_list_tmp = 3 * (i1 + j);
        V_data[3 * j] = V[C_list_tmp];
        V_data[3 * j + 1] = V[C_list_tmp + 1];
        V_data[3 * j + 2] = V[C_list_tmp + 2];
      }

      b_V_data.set((&V_data[0]), 3, m_tmp);
      coder::internal::blas::mtimes(b_V_data, S, b_y_tmp);
      for (j = 0; j < 3; j++) {
        d = b_y_tmp[j];
        d1 = b_y_tmp[j + 3];
        d2 = b_y_tmp[j + 6];
        for (aoffset = 0; aoffset < 3; aoffset++) {
          C_list_tmp = j + 3 * aoffset;
          K[C_list_tmp] = -(Muut_tmp[C_list_tmp] + ((d * Muxt[3 * aoffset] + d1 *
            Muxt[3 * aoffset + 1]) + d2 * Muxt[3 * aoffset + 2]));
        }
      }

      coder::b_pinv(e_y_tmp, b_b);
      V_size[0] = 3;
      V_size[1] = loop_ub;
      for (j = 0; j < loop_ub; j++) {
        V_data[3 * j] = V[3 * j];
        C_list_tmp = 3 * j + 1;
        V_data[C_list_tmp] = V[C_list_tmp];
        C_list_tmp = 3 * j + 2;
        V_data[C_list_tmp] = V[C_list_tmp];
      }

      coder::internal::blas::mtimes(V_data, V_size, b_b, Nut);
      input_sizes_idx_0 = Nut.size(1);
      b_C[0] = 0.0;
      b_C[1] = 0.0;
      b_C[2] = 0.0;
      for (boffset = 0; boffset < input_sizes_idx_0; boffset++) {
        aoffset = boffset * 3;
        b_C[0] += Nut[aoffset] * nlt[boffset];
        b_C[1] += Nut[aoffset + 1] * nlt[boffset];
        b_C[2] += Nut[aoffset + 2] * nlt[boffset];
      }

      for (j = 0; j < m_tmp; j++) {
        C_list_tmp = 3 * (i1 + j);
        V_data[3 * j] = V[C_list_tmp];
        V_data[3 * j + 1] = V[C_list_tmp + 1];
        V_data[3 * j + 2] = V[C_list_tmp + 2];
      }

      c_V_data.set((&V_data[0]), 3, m_tmp);
      coder::internal::blas::mtimes(c_V_data, S, Muut_tmp);
      d = mult[0];
      d1 = mult[1];
      d2 = mult[2];
      for (i1 = 0; i1 < 3; i1++) {
        k[i1] = -(b_C[i1] + ((Muut_tmp[i1] * d + Muut_tmp[i1 + 3] * d1) +
                             Muut_tmp[i1 + 6] * d2));
      }

      loop_ub = k_list.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        k_list[i1 + k_list.size(0) * (static_cast<int>(c_i) - 1)] = k[i1];
      }

      C_list_tmp = K_list.size(0);
      input_sizes_idx_0 = K_list.size(0);
      loop_ub = K_list.size(1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (j = 0; j < C_list_tmp; j++) {
          K_list[(j + K_list.size(0) * i1) + K_list.size(0) * K_list.size(1) * (
            static_cast<int>(c_i) - 1)] = K[j + input_sizes_idx_0 * i1];
        }
      }
    }

    //  remove redudant terms, the paragraph below equation 21
    if (hlt2.size(0) != 0) {
      C_list_tmp = hlt2.size(0);
    } else if ((Hxt2.size(0) != 0) && (Hxt2.size(1) != 0)) {
      C_list_tmp = Hxt2.size(0);
    } else {
      C_list_tmp = 0;
      if (Hxt2.size(0) > 0) {
        C_list_tmp = Hxt2.size(0);
      }
    }

    empty_non_axis_sizes = (C_list_tmp == 0);
    if (empty_non_axis_sizes || (hlt2.size(0) != 0)) {
      i2 = 1;
    } else {
      i2 = 0;
    }

    if (empty_non_axis_sizes || ((Hxt2.size(0) != 0) && (Hxt2.size(1) != 0))) {
      loop_ub = Hxt2.size(1);
    } else {
      loop_ub = 0;
    }

    Nut.set_size(C_list_tmp, (i2 + loop_ub));
    input_sizes_idx_0 = i2;
    for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
      for (j = 0; j < C_list_tmp; j++) {
        Nut[j] = hlt2[j];
      }
    }

    for (i1 = 0; i1 < loop_ub; i1++) {
      for (j = 0; j < C_list_tmp; j++) {
        Nut[j + Nut.size(0) * (i1 + i2)] = Hxt2[j + C_list_tmp * i1];
      }
    }

    coder::svd(Nut, U, Hxt2, unusedU1);
    Hxt2.set_size(Nut.size(0), Nut.size(1));
    loop_ub = Nut.size(0) * Nut.size(1) - 1;
    for (i1 = 0; i1 <= loop_ub; i1++) {
      Hxt2[i1] = Nut[i1];
    }

    coder::internal::blas::mtimes(U, Hxt2, Nut);
    d = static_cast<double>(Nut.size(0)) - static_cast<double>(n);
    if (1.0 > d) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }

    C_list_tmp = Nut.size(1) - 1;
    Hxt2.set_size(loop_ub, Nut.size(1));
    for (i1 = 0; i1 <= C_list_tmp; i1++) {
      for (j = 0; j < loop_ub; j++) {
        Hxt2[j + Hxt2.size(0) * i1] = Nut[j + Nut.size(0) * i1];
      }
    }

    Nut.set_size(Hxt2.size(0), Hxt2.size(1));
    loop_ub = Hxt2.size(0) * Hxt2.size(1);
    for (i1 = 0; i1 < loop_ub; i1++) {
      Nut[i1] = Hxt2[i1];
    }

    loop_ub = Nut.size(0);
    hlt2.set_size(Nut.size(0));
    for (i1 = 0; i1 < loop_ub; i1++) {
      hlt2[i1] = Nut[i1];
    }

    if (2 > Nut.size(1)) {
      i1 = 0;
      j = 0;
    } else {
      i1 = 1;
      j = Nut.size(1);
    }

    loop_ub = Nut.size(0);
    input_sizes_idx_0 = j - i1;
    Hxt2.set_size(Nut.size(0), input_sizes_idx_0);
    for (j = 0; j < input_sizes_idx_0; j++) {
      for (aoffset = 0; aoffset < loop_ub; aoffset++) {
        Hxt2[aoffset + Hxt2.size(0) * j] = Nut[aoffset + Nut.size(0) * (i1 + j)];
      }
    }

    //  update constraint to go , equation 20 and 21
    //  update cost to go, equation 24 and 25
    for (i1 = 0; i1 < 3; i1++) {
      Muut_tmp[3 * i1] = K[i1];
      Muut_tmp[3 * i1 + 1] = K[i1 + 3];
      Muut_tmp[3 * i1 + 2] = K[i1 + 6];
    }

    for (i1 = 0; i1 < 3; i1++) {
      d = y_tmp[i1];
      d1 = y_tmp[i1 + 3];
      d2 = y_tmp[i1 + 6];
      d3 = Muut_tmp[i1];
      d4 = Muut_tmp[i1 + 3];
      d5 = Muut_tmp[i1 + 6];
      for (j = 0; j < 3; j++) {
        aoffset = 3 * j + 1;
        boffset = 3 * j + 2;
        input_sizes_idx_0 = i1 + 3 * j;
        d_y_tmp[input_sizes_idx_0] = (d * Vxxt[3 * j] + d1 * Vxxt[aoffset]) + d2
          * Vxxt[boffset];
        b_y_tmp[input_sizes_idx_0] = (d3 * Muut[3 * j] + d4 * Muut[aoffset]) +
          d5 * Muut[boffset];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      double d6;
      double d7;
      d = d_y_tmp[i1];
      d1 = d_y_tmp[i1 + 3];
      d2 = d_y_tmp[i1 + 6];
      d3 = Muut_tmp[i1];
      d4 = Muut_tmp[i1 + 3];
      d5 = Muut_tmp[i1 + 6];
      c_i = b_y_tmp[i1];
      d6 = b_y_tmp[i1 + 3];
      d7 = b_y_tmp[i1 + 6];
      for (j = 0; j < 3; j++) {
        aoffset = 3 * j + S_tmp;
        boffset = 3 * j + 1;
        C_list_tmp = 3 * j + 2;
        input_sizes_idx_0 = i1 + 3 * j;
        Vxxt[input_sizes_idx_0] = ((param->Q[input_sizes_idx_0] + ((d *
          A_list[aoffset] + d1 * A_list[aoffset + 1]) + d2 * A_list[aoffset + 2]))
          + ((2.0 * d3 * Muxt[3 * j] + 2.0 * d4 * Muxt[boffset]) + 2.0 * d5 *
             Muxt[C_list_tmp])) + ((c_i * K[3 * j] + d6 * K[boffset]) + d7 *
          K[C_list_tmp]);
      }
    }

    //      Vxxt = Mxxt + 2*Muxt'*K + K'*Muut*K;
    for (i1 = 0; i1 < 3; i1++) {
      c_y_tmp[3 * i1] = (Vxxt[3 * i1] + Vxxt[i1]) / 2.0;
      input_sizes_idx_0 = 3 * i1 + 1;
      c_y_tmp[input_sizes_idx_0] = (Vxxt[input_sizes_idx_0] + Vxxt[i1 + 3]) /
        2.0;
      input_sizes_idx_0 = 3 * i1 + 2;
      c_y_tmp[input_sizes_idx_0] = (Vxxt[input_sizes_idx_0] + Vxxt[i1 + 6]) /
        2.0;
    }

    std::memcpy(&Vxxt[0], &c_y_tmp[0], 9U * sizeof(double));

    //  this is an astonishingly important step, I debugged almost a day to figure out 
    d = mult[0];
    d1 = mult[1];
    d2 = mult[2];
    d3 = vxlt[0];
    d4 = vxlt[1];
    d5 = vxlt[2];
    for (i1 = 0; i1 < 3; i1++) {
      c_y_tmp[3 * i1] = Muxt[i1] + b_y_tmp[3 * i1];
      input_sizes_idx_0 = 3 * i1 + 1;
      c_y_tmp[input_sizes_idx_0] = Muxt[i1 + 3] + b_y_tmp[input_sizes_idx_0];
      input_sizes_idx_0 = 3 * i1 + 2;
      c_y_tmp[input_sizes_idx_0] = Muxt[i1 + 6] + b_y_tmp[input_sizes_idx_0];
      b_C[i1] = (Muut_tmp[i1] * d + Muut_tmp[i1 + 3] * d1) + Muut_tmp[i1 + 6] *
        d2;
      mult[i1] = (y_tmp[i1] * d3 + y_tmp[i1 + 3] * d4) + y_tmp[i1 + 6] * d5;
    }

    d = k[0];
    d1 = k[1];
    d2 = k[2];
    for (i1 = 0; i1 < 3; i1++) {
      vxlt[i1] = (mult[i1] + b_C[i1]) + ((c_y_tmp[i1] * d + c_y_tmp[i1 + 3] * d1)
        + c_y_tmp[i1 + 6] * d2);
    }

    //  update
    //      Hxt1 = Hxt;
    //      hlt1 = hlt;
  }

  //  a final forward pass to get x_list
  x_list.set_size((static_cast<int>(param->nx)), (static_cast<int>(param->N +
    1.0)));
  loop_ub = static_cast<int>(param->nx) * static_cast<int>(param->N + 1.0);
  for (i = 0; i < loop_ub; i++) {
    x_list[i] = 0.0;
  }

  //  from 0 to N
  loop_ub = static_cast<int>(param->nx);
  for (i = 0; i < loop_ub; i++) {
    x_list[i] = param->x0[i];
  }

  i = static_cast<int>(param->N);
  if (0 <= i - 1) {
    mc = K_list.size(0) - 1;
    inner = K_list.size(1) - 1;
  }

  for (b_i = 0; b_i < i; b_i++) {
    C.set_size(K_list.size(0));
    for (d_i = 0; d_i <= mc; d_i++) {
      C[d_i] = 0.0;
    }

    for (boffset = 0; boffset <= inner; boffset++) {
      aoffset = boffset * K_list.size(0);
      for (d_i = 0; d_i <= mc; d_i++) {
        i1 = aoffset + d_i;
        C[d_i] = C[d_i] + K_list[(i1 % K_list.size(0) + K_list.size(0) * (i1 /
          K_list.size(0))) + K_list.size(0) * K_list.size(1) * b_i] *
          x_list[boffset + x_list.size(0) * b_i];
      }
    }

    C_list_tmp = x_list.size(0) - 1;
    hlt2.set_size(x_list.size(0));
    for (i1 = 0; i1 <= C_list_tmp; i1++) {
      hlt2[i1] = x_list[i1 + x_list.size(0) * b_i];
    }

    loop_ub = C.size(0);
    for (i1 = 0; i1 < loop_ub; i1++) {
      C[i1] = C[i1] + k_list[i1 + k_list.size(0) * b_i];
    }

    for (i1 = 0; i1 < 3; i1++) {
      j = i1 + 9 * b_i;
      b_C[i1] = ((A_list[j] * hlt2[0] + A_list[j + 3] * hlt2[1]) + A_list[j + 6]
                 * hlt2[2]) + ((B_list[j] * C[0] + B_list[j + 3] * C[1]) +
        B_list[j + 6] * C[2]);
    }

    loop_ub = x_list.size(0);
    for (i1 = 0; i1 < loop_ub; i1++) {
      x_list[i1 + x_list.size(0) * (b_i + 1)] = b_C[i1];
    }
  }

  coder::toc();

  //  nSoln = N;
  //  Soln(nSoln+1).K = zeros(nx,nu);
  //  Soln(nSoln+1).k = zeros(nu,1);
  //  Soln(nSoln+1).x = zeros(nx,1);
  //
  //  for i=1:nSoln
  //      Soln(i).K = K_list(:,:,i);
  //      Soln(i).k = k_list(:,i);
  //      Soln(i).x = x_list(:,i);
  //  end
  //  Soln(N+1).x = x_list(:,N+1);
}

//
// File trailer for ecLQR_laine.cpp
//
// [EOF]
//
