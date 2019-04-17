/*
 * File: xzhgeqz.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "xzhgeqz.h"
#include "CalScanLineParam_emxutil.h"
#include "xzlartg.h"
#include "sqrt.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *A
 *                int ilo
 *                int ihi
 *                int *info
 *                emxArray_creal_T *alpha1
 *                emxArray_creal_T *beta1
 * Return Type  : void
 */
void xzhgeqz(const emxArray_creal_T *A, int ilo, int ihi, int *info,
             emxArray_creal_T *alpha1, emxArray_creal_T *beta1)
{
  emxArray_creal_T *b_A;
  int jm1;
  int jp1;
  double eshift_re;
  double eshift_im;
  creal_T ctemp;
  double anorm;
  double scale;
  double reAij;
  double sumsq;
  double b_atol;
  boolean_T firstNonZero;
  int j;
  double ascale;
  double bscale;
  int i;
  double imAij;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  int ifirst;
  int istart;
  double temp2;
  int ilast;
  int ilastm1;
  int ifrstm;
  int ilastm;
  int iiter;
  boolean_T goto60;
  boolean_T goto70;
  boolean_T goto90;
  int jiter;
  int exitg1;
  boolean_T b_guard1 = false;
  boolean_T guard3 = false;
  boolean_T exitg2;
  creal_T b_ascale;
  creal_T shift;
  creal_T c_A;
  double ad22_re;
  double ad22_im;
  double t1_im;
  emxInit_creal_T1(&b_A, 2);
  jm1 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity_creal_T1(b_A, jm1);
  jp1 = A->size[0] * A->size[1];
  for (jm1 = 0; jm1 < jp1; jm1++) {
    b_A->data[jm1] = A->data[jm1];
  }

  *info = -1;
  if ((A->size[0] == 1) && (A->size[1] == 1)) {
    ihi = 1;
  }

  jm1 = alpha1->size[0];
  alpha1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(alpha1, jm1);
  jp1 = A->size[0];
  for (jm1 = 0; jm1 < jp1; jm1++) {
    alpha1->data[jm1].re = 0.0;
    alpha1->data[jm1].im = 0.0;
  }

  jm1 = beta1->size[0];
  beta1->size[0] = A->size[0];
  emxEnsureCapacity_creal_T(beta1, jm1);
  jp1 = A->size[0];
  for (jm1 = 0; jm1 < jp1; jm1++) {
    beta1->data[jm1].re = 1.0;
    beta1->data[jm1].im = 0.0;
  }

  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  anorm = 0.0;
  if (!(ilo > ihi)) {
    scale = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      jm1 = j + 1;
      if (ihi < j + 1) {
        jm1 = ihi;
      }

      for (i = ilo; i <= jm1; i++) {
        reAij = A->data[(i + A->size[0] * (j - 1)) - 1].re;
        imAij = A->data[(i + A->size[0] * (j - 1)) - 1].im;
        if (reAij != 0.0) {
          anorm = fabs(reAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = anorm;
            firstNonZero = false;
          } else if (scale < anorm) {
            temp2 = scale / anorm;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = anorm;
          } else {
            temp2 = anorm / scale;
            sumsq += temp2 * temp2;
          }
        }

        if (imAij != 0.0) {
          anorm = fabs(imAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = anorm;
            firstNonZero = false;
          } else if (scale < anorm) {
            temp2 = scale / anorm;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = anorm;
          } else {
            temp2 = anorm / scale;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    anorm = scale * sqrt(sumsq);
  }

  reAij = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (reAij > 2.2250738585072014E-308) {
    b_atol = reAij;
  }

  reAij = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    reAij = anorm;
  }

  ascale = 1.0 / reAij;
  bscale = 1.0 / sqrt(A->size[0]);
  firstNonZero = true;
  for (j = ihi; j < A->size[0]; j++) {
    alpha1->data[j] = A->data[j + A->size[0] * j];
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    ifrstm = ilo;
    ilastm = ihi;
    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 1;
    do {
      exitg1 = 0;
      if (jiter <= 30 * ((ihi - ilo) + 1)) {
        b_guard1 = false;
        if (ilast + 1 == ilo) {
          goto60 = true;
          b_guard1 = true;
        } else if (fabs(b_A->data[ilast + b_A->size[0] * ilastm1].re) + fabs
                   (b_A->data[ilast + b_A->size[0] * ilastm1].im) <= b_atol) {
          b_A->data[ilast + b_A->size[0] * ilastm1].re = 0.0;
          b_A->data[ilast + b_A->size[0] * ilastm1].im = 0.0;
          goto60 = true;
          b_guard1 = true;
        } else {
          j = ilastm1;
          guard3 = false;
          exitg2 = false;
          while ((!exitg2) && (j + 1 >= ilo)) {
            if (j + 1 == ilo) {
              guard3 = true;
              exitg2 = true;
            } else if (fabs(b_A->data[j + b_A->size[0] * (j - 1)].re) + fabs
                       (b_A->data[j + b_A->size[0] * (j - 1)].im) <= b_atol) {
              b_A->data[j + b_A->size[0] * (j - 1)].re = 0.0;
              b_A->data[j + b_A->size[0] * (j - 1)].im = 0.0;
              guard3 = true;
              exitg2 = true;
            } else {
              j--;
              guard3 = false;
            }
          }

          if (guard3) {
            ifirst = j + 1;
            goto70 = true;
          }

          if (goto70) {
            b_guard1 = true;
          } else {
            jp1 = alpha1->size[0];
            jm1 = alpha1->size[0];
            alpha1->size[0] = jp1;
            emxEnsureCapacity_creal_T(alpha1, jm1);
            for (jm1 = 0; jm1 < jp1; jm1++) {
              alpha1->data[jm1].re = rtNaN;
              alpha1->data[jm1].im = 0.0;
            }

            jp1 = beta1->size[0];
            jm1 = beta1->size[0];
            beta1->size[0] = jp1;
            emxEnsureCapacity_creal_T(beta1, jm1);
            for (jm1 = 0; jm1 < jp1; jm1++) {
              beta1->data[jm1].re = rtNaN;
              beta1->data[jm1].im = 0.0;
            }

            *info = 0;
            exitg1 = 1;
          }
        }

        if (b_guard1) {
          if (goto60) {
            goto60 = false;
            alpha1->data[ilast] = b_A->data[ilast + b_A->size[0] * ilast];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              firstNonZero = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0;
              eshift_im = 0.0;
              ilastm = ilast + 1;
              if (ifrstm > ilast + 1) {
                ifrstm = ilo;
              }

              jiter++;
            }
          } else {
            if (goto70) {
              goto70 = false;
              iiter++;
              ifrstm = ifirst;
              if (iiter - iiter / 10 * 10 != 0) {
                anorm = ascale * b_A->data[ilastm1 + b_A->size[0] * ilastm1].re;
                reAij = ascale * b_A->data[ilastm1 + b_A->size[0] * ilastm1].im;
                if (reAij == 0.0) {
                  shift.re = anorm / bscale;
                  shift.im = 0.0;
                } else if (anorm == 0.0) {
                  shift.re = 0.0;
                  shift.im = reAij / bscale;
                } else {
                  shift.re = anorm / bscale;
                  shift.im = reAij / bscale;
                }

                anorm = ascale * b_A->data[ilast + b_A->size[0] * ilast].re;
                reAij = ascale * b_A->data[ilast + b_A->size[0] * ilast].im;
                if (reAij == 0.0) {
                  ad22_re = anorm / bscale;
                  ad22_im = 0.0;
                } else if (anorm == 0.0) {
                  ad22_re = 0.0;
                  ad22_im = reAij / bscale;
                } else {
                  ad22_re = anorm / bscale;
                  ad22_im = reAij / bscale;
                }

                temp2 = 0.5 * (shift.re + ad22_re);
                t1_im = 0.5 * (shift.im + ad22_im);
                anorm = ascale * b_A->data[ilastm1 + b_A->size[0] * ilast].re;
                reAij = ascale * b_A->data[ilastm1 + b_A->size[0] * ilast].im;
                if (reAij == 0.0) {
                  sumsq = anorm / bscale;
                  imAij = 0.0;
                } else if (anorm == 0.0) {
                  sumsq = 0.0;
                  imAij = reAij / bscale;
                } else {
                  sumsq = anorm / bscale;
                  imAij = reAij / bscale;
                }

                anorm = ascale * b_A->data[ilast + b_A->size[0] * ilastm1].re;
                reAij = ascale * b_A->data[ilast + b_A->size[0] * ilastm1].im;
                if (reAij == 0.0) {
                  scale = anorm / bscale;
                  anorm = 0.0;
                } else if (anorm == 0.0) {
                  scale = 0.0;
                  anorm = reAij / bscale;
                } else {
                  scale = anorm / bscale;
                  anorm = reAij / bscale;
                }

                reAij = shift.re * ad22_im + shift.im * ad22_re;
                shift.re = ((temp2 * temp2 - t1_im * t1_im) + (sumsq * scale -
                  imAij * anorm)) - (shift.re * ad22_re - shift.im * ad22_im);
                shift.im = ((temp2 * t1_im + t1_im * temp2) + (sumsq * anorm +
                  imAij * scale)) - reAij;
                b_sqrt(&shift);
                if ((temp2 - ad22_re) * shift.re + (t1_im - ad22_im) * shift.im <=
                    0.0) {
                  shift.re += temp2;
                  shift.im += t1_im;
                } else {
                  shift.re = temp2 - shift.re;
                  shift.im = t1_im - shift.im;
                }
              } else {
                anorm = ascale * b_A->data[ilast + b_A->size[0] * ilastm1].re;
                reAij = ascale * b_A->data[ilast + b_A->size[0] * ilastm1].im;
                if (reAij == 0.0) {
                  sumsq = anorm / bscale;
                  imAij = 0.0;
                } else if (anorm == 0.0) {
                  sumsq = 0.0;
                  imAij = reAij / bscale;
                } else {
                  sumsq = anorm / bscale;
                  imAij = reAij / bscale;
                }

                eshift_re += sumsq;
                eshift_im += imAij;
                shift.re = eshift_re;
                shift.im = eshift_im;
              }

              j = ilastm1;
              jp1 = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > ifirst)) {
                istart = j + 1;
                ctemp.re = ascale * b_A->data[j + b_A->size[0] * j].re -
                  shift.re * bscale;
                ctemp.im = ascale * b_A->data[j + b_A->size[0] * j].im -
                  shift.im * bscale;
                anorm = fabs(ctemp.re) + fabs(ctemp.im);
                temp2 = ascale * (fabs(b_A->data[jp1 + b_A->size[0] * j].re) +
                                  fabs(b_A->data[jp1 + b_A->size[0] * j].im));
                reAij = anorm;
                if (temp2 > anorm) {
                  reAij = temp2;
                }

                if ((reAij < 1.0) && (reAij != 0.0)) {
                  anorm /= reAij;
                  temp2 /= reAij;
                }

                if ((fabs(b_A->data[j + b_A->size[0] * (j - 1)].re) + fabs
                     (b_A->data[j + b_A->size[0] * (j - 1)].im)) * temp2 <=
                    anorm * b_atol) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  jp1 = j;
                  j--;
                }
              }

              if (!goto90) {
                istart = ifirst;
                ctemp.re = ascale * b_A->data[(ifirst + b_A->size[0] * (ifirst -
                  1)) - 1].re - shift.re * bscale;
                ctemp.im = ascale * b_A->data[(ifirst + b_A->size[0] * (ifirst -
                  1)) - 1].im - shift.im * bscale;
                goto90 = true;
              }
            }

            if (goto90) {
              goto90 = false;
              b_ascale.re = ascale * b_A->data[istart + b_A->size[0] * (istart -
                1)].re;
              b_ascale.im = ascale * b_A->data[istart + b_A->size[0] * (istart -
                1)].im;
              b_xzlartg(ctemp, b_ascale, &imAij, &shift);
              j = istart;
              jm1 = istart - 2;
              while (j < ilast + 1) {
                if (j > istart) {
                  b_ascale = b_A->data[(j + b_A->size[0] * jm1) - 1];
                  c_A = b_A->data[j + b_A->size[0] * jm1];
                  xzlartg(b_ascale, c_A, &imAij, &shift, &b_A->data[(j +
                           b_A->size[0] * jm1) - 1]);
                  b_A->data[j + b_A->size[0] * jm1].re = 0.0;
                  b_A->data[j + b_A->size[0] * jm1].im = 0.0;
                }

                for (jp1 = j - 1; jp1 < ilastm; jp1++) {
                  anorm = shift.re * b_A->data[j + b_A->size[0] * jp1].re -
                    shift.im * b_A->data[j + b_A->size[0] * jp1].im;
                  reAij = shift.re * b_A->data[j + b_A->size[0] * jp1].im +
                    shift.im * b_A->data[j + b_A->size[0] * jp1].re;
                  ad22_re = imAij * b_A->data[(j + b_A->size[0] * jp1) - 1].re +
                    anorm;
                  ad22_im = imAij * b_A->data[(j + b_A->size[0] * jp1) - 1].im +
                    reAij;
                  anorm = b_A->data[(j + b_A->size[0] * jp1) - 1].re;
                  reAij = b_A->data[(j + b_A->size[0] * jp1) - 1].im;
                  scale = b_A->data[(j + b_A->size[0] * jp1) - 1].im;
                  sumsq = b_A->data[(j + b_A->size[0] * jp1) - 1].re;
                  b_A->data[j + b_A->size[0] * jp1].re = imAij * b_A->data[j +
                    b_A->size[0] * jp1].re - (shift.re * anorm + shift.im *
                    reAij);
                  b_A->data[j + b_A->size[0] * jp1].im = imAij * b_A->data[j +
                    b_A->size[0] * jp1].im - (shift.re * scale - shift.im *
                    sumsq);
                  b_A->data[(j + b_A->size[0] * jp1) - 1].re = ad22_re;
                  b_A->data[(j + b_A->size[0] * jp1) - 1].im = ad22_im;
                }

                shift.re = -shift.re;
                shift.im = -shift.im;
                jp1 = j;
                if (ilast + 1 < j + 2) {
                  jp1 = ilast - 1;
                }

                for (i = ifrstm - 1; i < jp1 + 2; i++) {
                  anorm = shift.re * b_A->data[i + b_A->size[0] * (j - 1)].re -
                    shift.im * b_A->data[i + b_A->size[0] * (j - 1)].im;
                  reAij = shift.re * b_A->data[i + b_A->size[0] * (j - 1)].im +
                    shift.im * b_A->data[i + b_A->size[0] * (j - 1)].re;
                  ad22_re = imAij * b_A->data[i + b_A->size[0] * j].re + anorm;
                  ad22_im = imAij * b_A->data[i + b_A->size[0] * j].im + reAij;
                  anorm = b_A->data[i + b_A->size[0] * j].re;
                  reAij = b_A->data[i + b_A->size[0] * j].im;
                  scale = b_A->data[i + b_A->size[0] * j].im;
                  sumsq = b_A->data[i + b_A->size[0] * j].re;
                  b_A->data[i + b_A->size[0] * (j - 1)].re = imAij * b_A->data[i
                    + b_A->size[0] * (j - 1)].re - (shift.re * anorm + shift.im *
                    reAij);
                  b_A->data[i + b_A->size[0] * (j - 1)].im = imAij * b_A->data[i
                    + b_A->size[0] * (j - 1)].im - (shift.re * scale - shift.im *
                    sumsq);
                  b_A->data[i + b_A->size[0] * j].re = ad22_re;
                  b_A->data[i + b_A->size[0] * j].im = ad22_im;
                }

                jm1 = j - 1;
                j++;
              }
            }

            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (firstNonZero) {
      *info = ilast;
      for (jp1 = 0; jp1 < ilast + 1; jp1++) {
        alpha1->data[jp1].re = rtNaN;
        alpha1->data[jp1].im = 0.0;
        beta1->data[jp1].re = rtNaN;
        beta1->data[jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    for (j = 0; j < ilo - 1; j++) {
      alpha1->data[j] = b_A->data[j + b_A->size[0] * j];
    }
  }

  emxFree_creal_T(&b_A);
  (*info)++;
}

/*
 * File trailer for xzhgeqz.c
 *
 * [EOF]
 */
