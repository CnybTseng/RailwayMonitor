/*
 * File: roots.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "roots.h"
#include "CalScanLineParam_emxutil.h"
#include "xzhseqr.h"
#include "xgehrd.h"
#include "anyNonFinite.h"
#include "xzgeev.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *c
 *                emxArray_creal_T *r
 * Return Type  : void
 */
void roots(const emxArray_real_T *c, emxArray_creal_T *r)
{
  int m;
  int k2;
  int k1;
  int nTrailingZeros;
  emxArray_real_T *ctmp;
  int companDim;
  boolean_T exitg1;
  int j;
  emxArray_creal_T *a;
  boolean_T exitg2;
  emxArray_creal_T *eiga;
  boolean_T p;
  emxArray_creal_T *beta1;
  int iv0[2];
  int exitg3;
  double a_re;
  double eiga_re;
  int jend;
  double a_im;
  double eiga_im;
  boolean_T b_a;
  double beta1_re;
  double beta1_im;
  double brm;
  m = r->size[0];
  r->size[0] = c->size[1] - 1;
  emxEnsureCapacity_creal_T(r, m);
  k2 = c->size[1];
  for (m = 0; m <= k2 - 2; m++) {
    r->data[m].re = 0.0;
    r->data[m].im = 0.0;
  }

  k1 = 1;
  while ((k1 <= c->size[1]) && (!(c->data[k1 - 1] != 0.0))) {
    k1++;
  }

  k2 = c->size[1];
  while ((k2 >= k1) && (!(c->data[k2 - 1] != 0.0))) {
    k2--;
  }

  nTrailingZeros = c->size[1] - k2;
  if (k1 < k2) {
    emxInit_real_T(&ctmp, 2);
    companDim = k2 - k1;
    m = ctmp->size[0] * ctmp->size[1];
    ctmp->size[0] = c->size[0];
    ctmp->size[1] = c->size[1];
    emxEnsureCapacity_real_T(ctmp, m);
    exitg1 = false;
    while ((!exitg1) && (companDim > 0)) {
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j + 1 <= companDim)) {
        ctmp->data[j] = c->data[k1 + j] / c->data[k1 - 1];
        if (rtIsInf(fabs(ctmp->data[j]))) {
          exitg2 = true;
        } else {
          j++;
        }
      }

      if (j + 1 > companDim) {
        exitg1 = true;
      } else {
        k1++;
        companDim--;
      }
    }

    if (companDim < 1) {
      m = r->size[0];
      if (1 > nTrailingZeros) {
        r->size[0] = 0;
      } else {
        r->size[0] = nTrailingZeros;
      }

      emxEnsureCapacity_creal_T(r, m);
    } else {
      emxInit_creal_T1(&a, 2);
      m = a->size[0] * a->size[1];
      a->size[0] = companDim;
      a->size[1] = companDim;
      emxEnsureCapacity_creal_T1(a, m);
      k2 = companDim * companDim;
      for (m = 0; m < k2; m++) {
        a->data[m].re = 0.0;
        a->data[m].im = 0.0;
      }

      for (k2 = 0; k2 < companDim - 1; k2++) {
        a->data[a->size[0] * k2].re = -ctmp->data[k2];
        a->data[a->size[0] * k2].im = 0.0;
        a->data[(k2 + a->size[0] * k2) + 1].re = 1.0;
        a->data[(k2 + a->size[0] * k2) + 1].im = 0.0;
      }

      a->data[a->size[0] * (companDim - 1)].re = -ctmp->data[companDim - 1];
      a->data[a->size[0] * (companDim - 1)].im = 0.0;
      for (k2 = 1; k2 <= nTrailingZeros; k2++) {
        r->data[k2 - 1].re = 0.0;
        r->data[k2 - 1].im = 0.0;
      }

      emxInit_creal_T(&eiga, 1);
      if (anyNonFinite(a)) {
        if ((a->size[0] == 1) && (a->size[1] == 1)) {
          m = eiga->size[0];
          eiga->size[0] = 1;
          emxEnsureCapacity_creal_T(eiga, m);
          eiga->data[0].re = rtNaN;
          eiga->data[0].im = 0.0;
        } else {
          m = eiga->size[0];
          eiga->size[0] = a->size[0];
          emxEnsureCapacity_creal_T(eiga, m);
          k2 = a->size[0];
          for (m = 0; m < k2; m++) {
            eiga->data[m].re = rtNaN;
            eiga->data[m].im = 0.0;
          }
        }
      } else if ((a->size[0] == 1) && (a->size[1] == 1)) {
        m = eiga->size[0];
        eiga->size[0] = 1;
        emxEnsureCapacity_creal_T(eiga, m);
        eiga->data[0] = a->data[0];
      } else {
        p = (a->size[0] == a->size[1]);
        if (p) {
          j = 0;
          exitg1 = false;
          while ((!exitg1) && (j <= a->size[1] - 1)) {
            k2 = 0;
            do {
              exitg3 = 0;
              if (k2 <= j) {
                a_re = a->data[j + a->size[0] * k2].re;
                a_im = -a->data[j + a->size[0] * k2].im;
                b_a = ((a->data[k2 + a->size[0] * j].re == a_re) && (a->data[k2
                        + a->size[0] * j].im == a_im));
                if (!b_a) {
                  p = false;
                  exitg3 = 1;
                } else {
                  k2++;
                }
              } else {
                j++;
                exitg3 = 2;
              }
            } while (exitg3 == 0);

            if (exitg3 == 1) {
              exitg1 = true;
            }
          }
        }

        if (p) {
          if (anyNonFinite(a)) {
            for (m = 0; m < 2; m++) {
              iv0[m] = a->size[m];
            }

            m = a->size[0] * a->size[1];
            a->size[0] = iv0[0];
            a->size[1] = iv0[1];
            emxEnsureCapacity_creal_T1(a, m);
            k2 = iv0[0] * iv0[1];
            for (m = 0; m < k2; m++) {
              a->data[m].re = rtNaN;
              a->data[m].im = 0.0;
            }

            m = a->size[0];
            if (!(1 >= a->size[0])) {
              k1 = 2;
              if (a->size[0] - 2 < a->size[1] - 1) {
                jend = a->size[0] - 1;
              } else {
                jend = a->size[1];
              }

              for (j = 1; j <= jend; j++) {
                for (k2 = k1; k2 <= m; k2++) {
                  a->data[(k2 + a->size[0] * (j - 1)) - 1].re = 0.0;
                  a->data[(k2 + a->size[0] * (j - 1)) - 1].im = 0.0;
                }

                k1++;
              }
            }
          } else {
            xgehrd(a);
            eml_zlahqr(a);
            m = a->size[0];
            if (!(3 >= a->size[0])) {
              k1 = 4;
              if (a->size[0] - 4 < a->size[1] - 1) {
                jend = a->size[0] - 3;
              } else {
                jend = a->size[1];
              }

              for (j = 1; j <= jend; j++) {
                for (k2 = k1; k2 <= m; k2++) {
                  a->data[(k2 + a->size[0] * (j - 1)) - 1].re = 0.0;
                  a->data[(k2 + a->size[0] * (j - 1)) - 1].im = 0.0;
                }

                if (j >= 1) {
                  k1++;
                }
              }
            }
          }

          m = eiga->size[0];
          eiga->size[0] = a->size[0];
          emxEnsureCapacity_creal_T(eiga, m);
          for (k2 = 0; k2 < a->size[0]; k2++) {
            eiga->data[k2] = a->data[k2 + a->size[0] * k2];
          }
        } else {
          emxInit_creal_T(&beta1, 1);
          xzgeev(a, &k2, eiga, beta1);
          m = eiga->size[0];
          emxEnsureCapacity_creal_T(eiga, m);
          k2 = eiga->size[0];
          for (m = 0; m < k2; m++) {
            eiga_re = eiga->data[m].re;
            eiga_im = eiga->data[m].im;
            beta1_re = beta1->data[m].re;
            beta1_im = beta1->data[m].im;
            if (beta1_im == 0.0) {
              if (eiga_im == 0.0) {
                eiga->data[m].re = eiga_re / beta1_re;
                eiga->data[m].im = 0.0;
              } else if (eiga_re == 0.0) {
                eiga->data[m].re = 0.0;
                eiga->data[m].im = eiga_im / beta1_re;
              } else {
                eiga->data[m].re = eiga_re / beta1_re;
                eiga->data[m].im = eiga_im / beta1_re;
              }
            } else if (beta1_re == 0.0) {
              if (eiga_re == 0.0) {
                eiga->data[m].re = eiga_im / beta1_im;
                eiga->data[m].im = 0.0;
              } else if (eiga_im == 0.0) {
                eiga->data[m].re = 0.0;
                eiga->data[m].im = -(eiga_re / beta1_im);
              } else {
                eiga->data[m].re = eiga_im / beta1_im;
                eiga->data[m].im = -(eiga_re / beta1_im);
              }
            } else {
              brm = fabs(beta1_re);
              a_re = fabs(beta1_im);
              if (brm > a_re) {
                a_im = beta1_im / beta1_re;
                a_re = beta1_re + a_im * beta1_im;
                eiga->data[m].re = (eiga_re + a_im * eiga_im) / a_re;
                eiga->data[m].im = (eiga_im - a_im * eiga_re) / a_re;
              } else if (a_re == brm) {
                if (beta1_re > 0.0) {
                  a_im = 0.5;
                } else {
                  a_im = -0.5;
                }

                if (beta1_im > 0.0) {
                  a_re = 0.5;
                } else {
                  a_re = -0.5;
                }

                eiga->data[m].re = (eiga_re * a_im + eiga_im * a_re) / brm;
                eiga->data[m].im = (eiga_im * a_im - eiga_re * a_re) / brm;
              } else {
                a_im = beta1_re / beta1_im;
                a_re = beta1_im + a_im * beta1_re;
                eiga->data[m].re = (a_im * eiga_re + eiga_im) / a_re;
                eiga->data[m].im = (a_im * eiga_im - eiga_re) / a_re;
              }
            }
          }

          emxFree_creal_T(&beta1);
        }
      }

      emxFree_creal_T(&a);
      for (k2 = 0; k2 < companDim; k2++) {
        r->data[k2 + nTrailingZeros] = eiga->data[k2];
      }

      emxFree_creal_T(&eiga);
      k2 = nTrailingZeros + companDim;
      m = r->size[0];
      if (1 > k2) {
        r->size[0] = 0;
      } else {
        r->size[0] = k2;
      }

      emxEnsureCapacity_creal_T(r, m);
    }

    emxFree_real_T(&ctmp);
  } else {
    m = r->size[0];
    if (1 > nTrailingZeros) {
      r->size[0] = 0;
    } else {
      r->size[0] = nTrailingZeros;
    }

    emxEnsureCapacity_creal_T(r, m);
  }
}

/*
 * File trailer for roots.c
 *
 * [EOF]
 */
