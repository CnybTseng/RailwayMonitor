/*
 * File: xscal.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "xscal.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                const creal_T a
 *                emxArray_creal_T *x
 *                int ix0
 *                int incx
 * Return Type  : void
 */
void b_xscal(int n, const creal_T a, emxArray_creal_T *x, int ix0, int incx)
{
  int i3;
  int k;
  double x_re;
  double x_im;
  i3 = ix0 + incx * (n - 1);
  for (k = ix0; k <= i3; k += incx) {
    x_re = x->data[k - 1].re;
    x_im = x->data[k - 1].im;
    x->data[k - 1].re = a.re * x_re - a.im * x_im;
    x->data[k - 1].im = a.re * x_im + a.im * x_re;
  }
}

/*
 * Arguments    : int n
 *                const creal_T a
 *                emxArray_creal_T *x
 *                int ix0
 * Return Type  : void
 */
void xscal(int n, const creal_T a, emxArray_creal_T *x, int ix0)
{
  int i1;
  int k;
  double x_re;
  double x_im;
  i1 = (ix0 + n) - 1;
  for (k = ix0; k <= i1; k++) {
    x_re = x->data[k - 1].re;
    x_im = x->data[k - 1].im;
    x->data[k - 1].re = a.re * x_re - a.im * x_im;
    x->data[k - 1].im = a.re * x_im + a.im * x_re;
  }
}

/*
 * File trailer for xscal.c
 *
 * [EOF]
 */
