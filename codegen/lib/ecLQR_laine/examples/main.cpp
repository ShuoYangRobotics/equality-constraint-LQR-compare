//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Include Files
#include "main.h"
#include "ecLQR_laine.h"
#include "ecLQR_laine_terminate.h"
#include "ecLQR_laine_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Declarations
static void argInit_3x100_real_T(double result[300]);
static void argInit_3x1_real_T(double result[3]);
static void argInit_3x3_real_T(double result[9]);
static void argInit_3x3x100_real_T(double result[900]);
static double argInit_real_T();
static void argInit_struct0_T(struct0_T *result);
static void main_ecLQR_laine();

// Function Definitions
//
// Arguments    : double result[300]
// Return Type  : void
//
static void argInit_3x100_real_T(double result[300])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 3; idx0++) {
    for (int idx1 = 0; idx1 < 100; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[9]
// Return Type  : void
//
static void argInit_3x3_real_T(double result[9])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 3; idx0++) {
    for (int idx1 = 0; idx1 < 3; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[900]
// Return Type  : void
//
static void argInit_3x3x100_real_T(double result[900])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 3; idx0++) {
    for (int idx1 = 0; idx1 < 3; idx1++) {
      for (int idx2 = 0; idx2 < 100; idx2++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[(idx0 + 3 * idx1) + 9 * idx2] = argInit_real_T();
      }
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : struct0_T *result
// Return Type  : void
//
static void argInit_struct0_T(struct0_T *result)
{
  double result_tmp;

  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result->nu = result_tmp;
  result->ncxu = result_tmp;
  result->ncx = result_tmp;
  argInit_3x1_real_T(result->x0);
  result->LQR_time = result_tmp;
  result->dt = result_tmp;
  result->N = result_tmp;
  argInit_3x3_real_T(result->A);
  result->simulation_noise = result_tmp;
  result->Cxu = result_tmp;
  result->nx = result_tmp;
  result->xN[0] = result->x0[0];
  result->xN[1] = result->x0[1];
  result->xN[2] = result->x0[2];
  for (int i = 0; i < 9; i++) {
    result_tmp = result->A[i];
    result->B[i] = result_tmp;
    result->Q[i] = result_tmp;
    result->R[i] = result_tmp;
    result->Qf[i] = result_tmp;
  }
}

//
// Arguments    : void
// Return Type  : void
//
static void main_ecLQR_laine()
{
  coder::array<double, 3U> K_list;
  coder::array<double, 2U> k_list;
  coder::array<double, 2U> x_list;
  struct0_T r;
  double A_list_tmp[900];
  double r_list_tmp[300];
  double dv[3];

  // Initialize function 'ecLQR_laine' input arguments.
  // Initialize function input argument 'param'.
  // Initialize function input argument 'xN'.
  // Initialize function input argument 'A_list'.
  argInit_3x3x100_real_T(A_list_tmp);

  // Initialize function input argument 'B_list'.
  // Initialize function input argument 'C_list'.
  // Initialize function input argument 'D_list'.
  // Initialize function input argument 'G_list'.
  // Initialize function input argument 'r_list'.
  argInit_3x100_real_T(r_list_tmp);

  // Initialize function input argument 'h_list'.
  // Call the entry-point 'ecLQR_laine'.
  argInit_struct0_T(&r);
  argInit_3x1_real_T(dv);
  ecLQR_laine(&r, dv, A_list_tmp, A_list_tmp, A_list_tmp, A_list_tmp, A_list_tmp,
              r_list_tmp, r_list_tmp, x_list, K_list, k_list);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. 
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_ecLQR_laine();

  // Terminate the application.
  // You do not need to do this more than one time.
  ecLQR_laine_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
