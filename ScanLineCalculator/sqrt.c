/*
 * File: sqrt.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "sqrt.h"
#include "xzhseqr.h"
#include "CalScanLineParam_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : creal_T *x
 * Return Type  : void
 */
void b_sqrt(creal_T *x)
{
  double xr;
  double xi;
  double absxi;
  double absxr;
  xr = x->re;
  xi = x->im;
  if (xi == 0.0) {
    if (xr < 0.0) {
      absxi = 0.0;
      xr = sqrt(-xr);
    } else {
      absxi = sqrt(xr);
      xr = 0.0;
    }
  } else if (xr == 0.0) {
    if (xi < 0.0) {
      absxi = sqrt(-xi / 2.0);
      xr = -absxi;
    } else {
      absxi = sqrt(xi / 2.0);
      xr = absxi;
    }
  } else if (rtIsNaN(xr)) {
    absxi = xr;
  } else if (rtIsNaN(xi)) {
    absxi = xi;
    xr = xi;
  } else if (rtIsInf(xi)) {
    absxi = fabs(xi);
    xr = xi;
  } else if (rtIsInf(xr)) {
    if (xr < 0.0) {
      absxi = 0.0;
      xr = xi * -xr;
    } else {
      absxi = xr;
      xr = 0.0;
    }
  } else {
    absxr = fabs(xr);
    absxi = fabs(xi);
    if ((absxr > 4.4942328371557893E+307) || (absxi > 4.4942328371557893E+307))
    {
      absxr *= 0.5;
      absxi *= 0.5;
      absxi = rt_hypotd_snf(absxr, absxi);
      if (absxi > absxr) {
        absxi = sqrt(absxi) * sqrt(1.0 + absxr / absxi);
      } else {
        absxi = sqrt(absxi) * 1.4142135623730951;
      }
    } else {
      absxi = sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
    }

    if (xr > 0.0) {
      xr = 0.5 * (xi / absxi);
    } else {
      if (xi < 0.0) {
        xr = -absxi;
      } else {
        xr = absxi;
      }

      absxi = 0.5 * (xi / xr);
    }
  }

  x->re = absxi;
  x->im = xr;
}

/*
 * File trailer for sqrt.c
 *
 * [EOF]
 */
