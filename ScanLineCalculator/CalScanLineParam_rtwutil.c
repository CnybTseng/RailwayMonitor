/*
 * File: CalScanLineParam_rtwutil.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "CalScanLineParam_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * File trailer for CalScanLineParam_rtwutil.c
 *
 * [EOF]
 */
