//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: timeKeeper.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 24-Mar-2021 09:42:45
//

// Include Files
#include "timeKeeper.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"

// Type Definitions
struct struct_T
{
  double tv_sec;
  double tv_nsec;
};

// Variable Definitions
static struct_T savedTime;
static boolean_T savedTime_not_empty;

// Function Definitions
//
// Arguments    : double newTime_tv_sec
//                double newTime_tv_nsec
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    namespace time
    {
      namespace impl
      {
        void timeKeeper(double newTime_tv_sec, double newTime_tv_nsec)
        {
          struct timespec b_timespec;
          if (!savedTime_not_empty) {
            clock_gettime(CLOCK_MONOTONIC, &b_timespec);
            savedTime_not_empty = true;
          }

          savedTime.tv_sec = newTime_tv_sec;
          savedTime.tv_nsec = newTime_tv_nsec;
        }

        //
        // Arguments    : double *outTime_tv_sec
        //                double *outTime_tv_nsec
        // Return Type  : void
        //
        void timeKeeper(double *outTime_tv_sec, double *outTime_tv_nsec)
        {
          *outTime_tv_sec = savedTime.tv_sec;
          *outTime_tv_nsec = savedTime.tv_nsec;
        }

        //
        // Arguments    : void
        // Return Type  : void
        //
      }
    }
  }
}

void savedTime_not_empty_init()
{
  savedTime_not_empty = false;
}

//
// File trailer for timeKeeper.cpp
//
// [EOF]
//
