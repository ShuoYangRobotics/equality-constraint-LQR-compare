//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: tic.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "tic.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include "coder_posix_time.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
namespace coder
{
  void tic()
  {
    struct timespec b_timespec;
    clock_gettime(CLOCK_MONOTONIC, &b_timespec);
    internal::time::impl::timeKeeper((double)b_timespec.tv_sec, (double)
      b_timespec.tv_nsec);
  }
}

//
// File trailer for tic.cpp
//
// [EOF]
//
