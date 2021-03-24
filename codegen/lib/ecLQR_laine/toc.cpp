//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: toc.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "toc.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include "coder_posix_time.h"
#include <stdio.h>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
namespace coder
{
  void toc()
  {
    struct timespec b_timespec;
    double tstart_tv_nsec;
    double tstart_tv_sec;
    internal::time::impl::timeKeeper(&tstart_tv_sec, &tstart_tv_nsec);
    clock_gettime(CLOCK_MONOTONIC, &b_timespec);
    printf("Elapsed time is %f seconds\n", ((double)b_timespec.tv_sec -
            tstart_tv_sec) + ((double)b_timespec.tv_nsec - tstart_tv_nsec) /
           1.0E+9);
    fflush(stdout);
  }
}

//
// File trailer for toc.cpp
//
// [EOF]
//
