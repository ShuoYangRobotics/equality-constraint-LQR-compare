//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ecLQR_laine_initialize.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "ecLQR_laine_initialize.h"
#include "ecLQR_laine_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void ecLQR_laine_initialize()
{
  savedTime_not_empty_init();
  isInitialized_ecLQR_laine = true;
}

//
// File trailer for ecLQR_laine_initialize.cpp
//
// [EOF]
//
