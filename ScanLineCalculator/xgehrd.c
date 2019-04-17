/*
 * File: xgehrd.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "xgehrd.h"
#include "xscal.h"
#include "recip.h"
#include "xdlapy3.h"
#include "xnrm2.h"
#include "CalScanLineParam_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : emxArray_creal_T *a
 * Return Type  : void
 */
void xgehrd(emxArray_creal_T *a)
{
  emxArray_creal_T *tau;
  emxArray_creal_T *work;
  int n;
  int ia0;
  int i0;
  int i;
  int im1n;
  int in;
  creal_T alpha1;
  int c;
  double alpha1_re;
  double alpha1_im;
  double xnorm;
  double beta1;
  int jy;
  int knt;
  boolean_T b_tau;
  double ai;
  int lastv;
  int lastc;
  int k;
  boolean_T exitg1;
  creal_T b_c;
  int ix;
  int exitg2;
  emxInit_creal_T(&tau, 1);
  emxInit_creal_T(&work, 1);
  n = a->size[0];
  ia0 = a->size[0] - 1;
  i0 = tau->size[0];
  tau->size[0] = ia0;
  emxEnsureCapacity_creal_T(tau, i0);
  ia0 = a->size[0];
  i0 = work->size[0];
  work->size[0] = ia0;
  emxEnsureCapacity_creal_T(work, i0);
  for (i0 = 0; i0 < ia0; i0++) {
    work->data[i0].re = 0.0;
    work->data[i0].im = 0.0;
  }

  for (i = 0; i < n - 1; i++) {
    im1n = i * n + 2;
    in = (i + 1) * n;
    alpha1 = a->data[(i + a->size[0] * i) + 1];
    ia0 = i + 3;
    if (!(ia0 < n)) {
      ia0 = n;
    }

    ia0 += i * n;
    c = (n - i) - 2;
    alpha1_re = 0.0;
    alpha1_im = 0.0;
    if (!(c + 1 <= 0)) {
      xnorm = xnrm2(c, a, ia0);
      if ((xnorm != 0.0) || (a->data[(i + a->size[0] * i) + 1].im != 0.0)) {
        beta1 = xdlapy3(a->data[(i + a->size[0] * i) + 1].re, a->data[(i +
          a->size[0] * i) + 1].im, xnorm);
        if (a->data[(i + a->size[0] * i) + 1].re >= 0.0) {
          beta1 = -beta1;
        }

        if (fabs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          i0 = (ia0 + c) - 1;
          do {
            knt++;
            for (k = ia0; k <= i0; k++) {
              xnorm = a->data[k - 1].re;
              ai = a->data[k - 1].im;
              a->data[k - 1].re = 9.9792015476736E+291 * xnorm - 0.0 * ai;
              a->data[k - 1].im = 9.9792015476736E+291 * ai + 0.0 * xnorm;
            }

            beta1 *= 9.9792015476736E+291;
            alpha1.re *= 9.9792015476736E+291;
            alpha1.im *= 9.9792015476736E+291;
          } while (!(fabs(beta1) >= 1.0020841800044864E-292));

          beta1 = xdlapy3(alpha1.re, alpha1.im, xnrm2(c, a, ia0));
          if (alpha1.re >= 0.0) {
            beta1 = -beta1;
          }

          xnorm = beta1 - alpha1.re;
          if (0.0 - alpha1.im == 0.0) {
            alpha1_re = xnorm / beta1;
            alpha1_im = 0.0;
          } else if (xnorm == 0.0) {
            alpha1_re = 0.0;
            alpha1_im = (0.0 - alpha1.im) / beta1;
          } else {
            alpha1_re = xnorm / beta1;
            alpha1_im = (0.0 - alpha1.im) / beta1;
          }

          b_c.re = alpha1.re - beta1;
          b_c.im = alpha1.im;
          xscal(c, recip(b_c), a, ia0);
          for (k = 1; k <= knt; k++) {
            beta1 *= 1.0020841800044864E-292;
          }

          alpha1.re = beta1;
          alpha1.im = 0.0;
        } else {
          xnorm = beta1 - a->data[(i + a->size[0] * i) + 1].re;
          ai = 0.0 - a->data[(i + a->size[0] * i) + 1].im;
          if (ai == 0.0) {
            alpha1_re = xnorm / beta1;
            alpha1_im = 0.0;
          } else if (xnorm == 0.0) {
            alpha1_re = 0.0;
            alpha1_im = ai / beta1;
          } else {
            alpha1_re = xnorm / beta1;
            alpha1_im = ai / beta1;
          }

          b_c.re = a->data[(i + a->size[0] * i) + 1].re - beta1;
          b_c.im = a->data[(i + a->size[0] * i) + 1].im;
          xscal(c, recip(b_c), a, ia0);
          alpha1.re = beta1;
          alpha1.im = 0.0;
        }
      }
    }

    tau->data[i].re = alpha1_re;
    tau->data[i].im = alpha1_im;
    a->data[(i + a->size[0] * i) + 1].re = 1.0;
    a->data[(i + a->size[0] * i) + 1].im = 0.0;
    c = (n - i) - 3;
    jy = (i + im1n) - 1;
    b_tau = ((tau->data[i].re != 0.0) || (tau->data[i].im != 0.0));
    if (b_tau) {
      lastv = c + 2;
      ia0 = jy + c;
      exitg1 = false;
      while ((!exitg1) && (lastv > 0)) {
        b_tau = ((a->data[ia0 + 1].re == 0.0) && (a->data[ia0 + 1].im == 0.0));
        if (b_tau) {
          lastv--;
          ia0--;
        } else {
          exitg1 = true;
        }
      }

      lastc = n;
      exitg1 = false;
      while ((!exitg1) && (lastc > 0)) {
        ia0 = in + lastc;
        c = ia0;
        do {
          exitg2 = 0;
          if (c <= ia0 + (lastv - 1) * n) {
            b_tau = ((a->data[c - 1].re != 0.0) || (a->data[c - 1].im != 0.0));
            if (b_tau) {
              exitg2 = 1;
            } else {
              c += n;
            }
          } else {
            lastc--;
            exitg2 = 2;
          }
        } while (exitg2 == 0);

        if (exitg2 == 1) {
          exitg1 = true;
        }
      }
    } else {
      lastv = 0;
      lastc = 0;
    }

    if (lastv > 0) {
      if (lastc != 0) {
        for (ia0 = 1; ia0 <= lastc; ia0++) {
          work->data[ia0 - 1].re = 0.0;
          work->data[ia0 - 1].im = 0.0;
        }

        ix = jy;
        i0 = (in + n * (lastv - 1)) + 1;
        for (knt = in + 1; knt <= i0; knt += n) {
          b_c.re = a->data[ix].re - 0.0 * a->data[ix].im;
          b_c.im = a->data[ix].im + 0.0 * a->data[ix].re;
          ia0 = 0;
          k = (knt + lastc) - 1;
          for (c = knt; c <= k; c++) {
            xnorm = a->data[c - 1].re * b_c.re - a->data[c - 1].im * b_c.im;
            ai = a->data[c - 1].re * b_c.im + a->data[c - 1].im * b_c.re;
            work->data[ia0].re += xnorm;
            work->data[ia0].im += ai;
            ia0++;
          }

          ix++;
        }
      }

      alpha1_re = -tau->data[i].re;
      alpha1_im = -tau->data[i].im;
      if (!((alpha1_re == 0.0) && (alpha1_im == 0.0))) {
        ia0 = in;
        for (knt = 1; knt <= lastv; knt++) {
          b_tau = ((a->data[jy].re != 0.0) || (a->data[jy].im != 0.0));
          if (b_tau) {
            b_c.re = a->data[jy].re * alpha1_re + a->data[jy].im * alpha1_im;
            b_c.im = a->data[jy].re * alpha1_im - a->data[jy].im * alpha1_re;
            ix = 0;
            i0 = lastc + ia0;
            for (k = ia0; k < i0; k++) {
              xnorm = work->data[ix].re * b_c.re - work->data[ix].im * b_c.im;
              ai = work->data[ix].re * b_c.im + work->data[ix].im * b_c.re;
              a->data[k].re += xnorm;
              a->data[k].im += ai;
              ix++;
            }
          }

          jy++;
          ia0 += n;
        }
      }
    }

    c = (n - i) - 3;
    im1n = (i + im1n) - 1;
    jy = (i + in) + 2;
    alpha1_re = tau->data[i].re;
    alpha1_im = -tau->data[i].im;
    if ((alpha1_re != 0.0) || (alpha1_im != 0.0)) {
      lastv = c + 2;
      ia0 = im1n + c;
      exitg1 = false;
      while ((!exitg1) && (lastv > 0)) {
        b_tau = ((a->data[ia0 + 1].re == 0.0) && (a->data[ia0 + 1].im == 0.0));
        if (b_tau) {
          lastv--;
          ia0--;
        } else {
          exitg1 = true;
        }
      }

      lastc = (n - i) - 1;
      exitg1 = false;
      while ((!exitg1) && (lastc > 0)) {
        ia0 = jy + (lastc - 1) * n;
        c = ia0;
        do {
          exitg2 = 0;
          if (c <= (ia0 + lastv) - 1) {
            b_tau = ((a->data[c - 1].re != 0.0) || (a->data[c - 1].im != 0.0));
            if (b_tau) {
              exitg2 = 1;
            } else {
              c++;
            }
          } else {
            lastc--;
            exitg2 = 2;
          }
        } while (exitg2 == 0);

        if (exitg2 == 1) {
          exitg1 = true;
        }
      }
    } else {
      lastv = 0;
      lastc = 0;
    }

    if (lastv > 0) {
      if (lastc != 0) {
        for (ia0 = 1; ia0 <= lastc; ia0++) {
          work->data[ia0 - 1].re = 0.0;
          work->data[ia0 - 1].im = 0.0;
        }

        ia0 = 0;
        i0 = jy + n * (lastc - 1);
        for (knt = jy; knt <= i0; knt += n) {
          ix = im1n;
          b_c.re = 0.0;
          b_c.im = 0.0;
          k = (knt + lastv) - 1;
          for (c = knt - 1; c < k; c++) {
            b_c.re += a->data[c].re * a->data[ix].re + a->data[c].im * a->
              data[ix].im;
            b_c.im += a->data[c].re * a->data[ix].im - a->data[c].im * a->
              data[ix].re;
            ix++;
          }

          work->data[ia0].re += b_c.re - 0.0 * b_c.im;
          work->data[ia0].im += b_c.im + 0.0 * b_c.re;
          ia0++;
        }
      }

      alpha1_re = -alpha1_re;
      alpha1_im = -alpha1_im;
      if (!((alpha1_re == 0.0) && (alpha1_im == 0.0))) {
        ia0 = jy - 1;
        jy = 0;
        for (knt = 1; knt <= lastc; knt++) {
          b_tau = ((work->data[jy].re != 0.0) || (work->data[jy].im != 0.0));
          if (b_tau) {
            b_c.re = work->data[jy].re * alpha1_re + work->data[jy].im *
              alpha1_im;
            b_c.im = work->data[jy].re * alpha1_im - work->data[jy].im *
              alpha1_re;
            ix = im1n;
            i0 = lastv + ia0;
            for (k = ia0; k < i0; k++) {
              xnorm = a->data[ix].re * b_c.re - a->data[ix].im * b_c.im;
              ai = a->data[ix].re * b_c.im + a->data[ix].im * b_c.re;
              a->data[k].re += xnorm;
              a->data[k].im += ai;
              ix++;
            }
          }

          jy++;
          ia0 += n;
        }
      }
    }

    a->data[(i + a->size[0] * i) + 1] = alpha1;
  }

  emxFree_creal_T(&work);
  emxFree_creal_T(&tau);
}

/*
 * File trailer for xgehrd.c
 *
 * [EOF]
 */
