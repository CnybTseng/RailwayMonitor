/*
 * File: xnrm2.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "xnrm2.h"
#include "xzhseqr.h"
#include "CalScanLineParam_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                const emxArray_creal_T *x
 *                int ix0
 * Return Type  : double
 */
double xnrm2(int n, const emxArray_creal_T *x, int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (!(n < 1)) {
    if (n == 1) {
      y = rt_hypotd_snf(x->data[ix0 - 1].re, x->data[ix0 - 1].im);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x->data[k - 1].re);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0 + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }

        absxk = fabs(x->data[k - 1].im);
        if (absxk > scale) {
          t = scale / absxk;
          y = 1.0 + y * t * t;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/*
 * File trailer for xnrm2.c
 *
 * [EOF]
 */
