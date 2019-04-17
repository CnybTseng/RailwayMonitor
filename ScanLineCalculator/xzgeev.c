/*
 * File: xzgeev.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "xzgeev.h"
#include "CalScanLineParam_emxutil.h"
#include "xzlartg.h"
#include "xzhgeqz.h"
#include "xzhseqr.h"
#include "CalScanLineParam_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_creal_T *A
 *                int *info
 *                emxArray_creal_T *alpha1
 *                emxArray_creal_T *beta1
 * Return Type  : void
 */
void xzgeev(const emxArray_creal_T *A, int *info, emxArray_creal_T *alpha1,
            emxArray_creal_T *beta1)
{
  emxArray_creal_T *At;
  int nzcount;
  int ii;
  double anrm;
  boolean_T exitg1;
  double absxk;
  boolean_T ilascl;
  double anrmto;
  int ilo;
  double ctoc;
  int ihi;
  boolean_T notdone;
  int exitg3;
  double cfrom1;
  int i;
  int n;
  double cto1;
  int j;
  double mul;
  creal_T b_At;
  creal_T c_At;
  double c;
  creal_T atmp;
  boolean_T exitg4;
  int exitg2;
  boolean_T d_At;
  double stemp_re;
  emxInit_creal_T1(&At, 2);
  nzcount = At->size[0] * At->size[1];
  At->size[0] = A->size[0];
  At->size[1] = A->size[1];
  emxEnsureCapacity_creal_T1(At, nzcount);
  ii = A->size[0] * A->size[1];
  for (nzcount = 0; nzcount < ii; nzcount++) {
    At->data[nzcount] = A->data[nzcount];
  }

  *info = 0;
  anrm = 0.0;
  nzcount = At->size[0] * At->size[1];
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nzcount - 1)) {
    absxk = rt_hypotd_snf(At->data[ii].re, At->data[ii].im);
    if (rtIsNaN(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }

      ii++;
    }
  }

  if (!((!rtIsInf(anrm)) && (!rtIsNaN(anrm)))) {
    nzcount = alpha1->size[0];
    alpha1->size[0] = At->size[0];
    emxEnsureCapacity_creal_T(alpha1, nzcount);
    ii = At->size[0];
    for (nzcount = 0; nzcount < ii; nzcount++) {
      alpha1->data[nzcount].re = rtNaN;
      alpha1->data[nzcount].im = 0.0;
    }

    nzcount = beta1->size[0];
    beta1->size[0] = At->size[0];
    emxEnsureCapacity_creal_T(beta1, nzcount);
    ii = At->size[0];
    for (nzcount = 0; nzcount < ii; nzcount++) {
      beta1->data[nzcount].re = rtNaN;
      beta1->data[nzcount].im = 0.0;
    }
  } else {
    ilascl = false;
    anrmto = anrm;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      anrmto = 6.7178761075670888E-139;
      ilascl = true;
    } else {
      if (anrm > 1.4885657073574029E+138) {
        anrmto = 1.4885657073574029E+138;
        ilascl = true;
      }
    }

    if (ilascl) {
      absxk = anrm;
      ctoc = anrmto;
      notdone = true;
      while (notdone) {
        cfrom1 = absxk * 2.0041683600089728E-292;
        cto1 = ctoc / 4.9896007738368E+291;
        if ((cfrom1 > ctoc) && (ctoc != 0.0)) {
          mul = 2.0041683600089728E-292;
          absxk = cfrom1;
        } else if (cto1 > absxk) {
          mul = 4.9896007738368E+291;
          ctoc = cto1;
        } else {
          mul = ctoc / absxk;
          notdone = false;
        }

        ii = At->size[0] * At->size[1] - 1;
        nzcount = At->size[0] * At->size[1];
        emxEnsureCapacity_creal_T1(At, nzcount);
        for (nzcount = 0; nzcount <= ii; nzcount++) {
          At->data[nzcount].re *= mul;
          At->data[nzcount].im *= mul;
        }
      }
    }

    ilo = 0;
    ihi = At->size[0];
    if (At->size[0] <= 1) {
      ihi = 1;
    } else {
      do {
        exitg3 = 0;
        i = 0;
        j = 0;
        notdone = false;
        ii = ihi;
        exitg1 = false;
        while ((!exitg1) && (ii > 0)) {
          nzcount = 0;
          i = ii;
          j = ihi;
          n = 1;
          exitg4 = false;
          while ((!exitg4) && (n <= ihi)) {
            d_At = ((At->data[(ii + At->size[0] * (n - 1)) - 1].re != 0.0) ||
                    (At->data[(ii + At->size[0] * (n - 1)) - 1].im != 0.0));
            if (d_At || (ii == n)) {
              if (nzcount == 0) {
                j = n;
                nzcount = 1;
                n++;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              n++;
            }
          }

          if (nzcount < 2) {
            notdone = true;
            exitg1 = true;
          } else {
            ii--;
          }
        }

        if (!notdone) {
          exitg3 = 2;
        } else {
          n = At->size[0];
          if (i != ihi) {
            for (ii = 0; ii < n; ii++) {
              atmp = At->data[(i + At->size[0] * ii) - 1];
              At->data[(i + At->size[0] * ii) - 1] = At->data[(ihi + At->size[0]
                * ii) - 1];
              At->data[(ihi + At->size[0] * ii) - 1] = atmp;
            }
          }

          if (j != ihi) {
            for (ii = 0; ii < ihi; ii++) {
              atmp = At->data[ii + At->size[0] * (j - 1)];
              At->data[ii + At->size[0] * (j - 1)] = At->data[ii + At->size[0] *
                (ihi - 1)];
              At->data[ii + At->size[0] * (ihi - 1)] = atmp;
            }
          }

          ihi--;
          if (ihi == 1) {
            exitg3 = 1;
          }
        }
      } while (exitg3 == 0);

      if (exitg3 == 1) {
      } else {
        do {
          exitg2 = 0;
          i = 0;
          j = 0;
          notdone = false;
          n = ilo + 1;
          exitg1 = false;
          while ((!exitg1) && (n <= ihi)) {
            nzcount = 0;
            i = ihi;
            j = n;
            ii = ilo + 1;
            exitg4 = false;
            while ((!exitg4) && (ii <= ihi)) {
              d_At = ((At->data[(ii + At->size[0] * (n - 1)) - 1].re != 0.0) ||
                      (At->data[(ii + At->size[0] * (n - 1)) - 1].im != 0.0));
              if (d_At || (ii == n)) {
                if (nzcount == 0) {
                  i = ii;
                  nzcount = 1;
                  ii++;
                } else {
                  nzcount = 2;
                  exitg4 = true;
                }
              } else {
                ii++;
              }
            }

            if (nzcount < 2) {
              notdone = true;
              exitg1 = true;
            } else {
              n++;
            }
          }

          if (!notdone) {
            exitg2 = 1;
          } else {
            n = At->size[0];
            if (i != ilo + 1) {
              for (ii = ilo; ii < n; ii++) {
                atmp = At->data[(i + At->size[0] * ii) - 1];
                At->data[(i + At->size[0] * ii) - 1] = At->data[ilo + At->size[0]
                  * ii];
                At->data[ilo + At->size[0] * ii] = atmp;
              }
            }

            if (j != ilo + 1) {
              for (ii = 0; ii < ihi; ii++) {
                atmp = At->data[ii + At->size[0] * (j - 1)];
                At->data[ii + At->size[0] * (j - 1)] = At->data[ii + At->size[0]
                  * ilo];
                At->data[ii + At->size[0] * ilo] = atmp;
              }
            }

            ilo++;
            if (ilo + 1 == ihi) {
              exitg2 = 1;
            }
          }
        } while (exitg2 == 0);
      }
    }

    n = At->size[0];
    if ((!(At->size[0] <= 1)) && (!(ihi < ilo + 3))) {
      for (ii = ilo; ii + 1 < ihi - 1; ii++) {
        for (nzcount = ihi - 1; nzcount + 1 > ii + 2; nzcount--) {
          b_At = At->data[(nzcount + At->size[0] * ii) - 1];
          c_At = At->data[nzcount + At->size[0] * ii];
          xzlartg(b_At, c_At, &c, &atmp, &At->data[(nzcount + At->size[0] * ii)
                  - 1]);
          At->data[nzcount + At->size[0] * ii].re = 0.0;
          At->data[nzcount + At->size[0] * ii].im = 0.0;
          for (j = ii + 1; j < n; j++) {
            absxk = atmp.re * At->data[nzcount + At->size[0] * j].re - atmp.im *
              At->data[nzcount + At->size[0] * j].im;
            ctoc = atmp.re * At->data[nzcount + At->size[0] * j].im + atmp.im *
              At->data[nzcount + At->size[0] * j].re;
            stemp_re = c * At->data[(nzcount + At->size[0] * j) - 1].re + absxk;
            absxk = c * At->data[(nzcount + At->size[0] * j) - 1].im + ctoc;
            ctoc = At->data[(nzcount + At->size[0] * j) - 1].re;
            cfrom1 = At->data[(nzcount + At->size[0] * j) - 1].im;
            cto1 = At->data[(nzcount + At->size[0] * j) - 1].im;
            mul = At->data[(nzcount + At->size[0] * j) - 1].re;
            At->data[nzcount + At->size[0] * j].re = c * At->data[nzcount +
              At->size[0] * j].re - (atmp.re * ctoc + atmp.im * cfrom1);
            At->data[nzcount + At->size[0] * j].im = c * At->data[nzcount +
              At->size[0] * j].im - (atmp.re * cto1 - atmp.im * mul);
            At->data[(nzcount + At->size[0] * j) - 1].re = stemp_re;
            At->data[(nzcount + At->size[0] * j) - 1].im = absxk;
          }

          atmp.re = -atmp.re;
          atmp.im = -atmp.im;
          for (i = 0; i < ihi; i++) {
            absxk = atmp.re * At->data[i + At->size[0] * (nzcount - 1)].re -
              atmp.im * At->data[i + At->size[0] * (nzcount - 1)].im;
            ctoc = atmp.re * At->data[i + At->size[0] * (nzcount - 1)].im +
              atmp.im * At->data[i + At->size[0] * (nzcount - 1)].re;
            stemp_re = c * At->data[i + At->size[0] * nzcount].re + absxk;
            absxk = c * At->data[i + At->size[0] * nzcount].im + ctoc;
            ctoc = At->data[i + At->size[0] * nzcount].re;
            cfrom1 = At->data[i + At->size[0] * nzcount].im;
            cto1 = At->data[i + At->size[0] * nzcount].im;
            mul = At->data[i + At->size[0] * nzcount].re;
            At->data[i + At->size[0] * (nzcount - 1)].re = c * At->data[i +
              At->size[0] * (nzcount - 1)].re - (atmp.re * ctoc + atmp.im *
              cfrom1);
            At->data[i + At->size[0] * (nzcount - 1)].im = c * At->data[i +
              At->size[0] * (nzcount - 1)].im - (atmp.re * cto1 - atmp.im * mul);
            At->data[i + At->size[0] * nzcount].re = stemp_re;
            At->data[i + At->size[0] * nzcount].im = absxk;
          }
        }
      }
    }

    xzhgeqz(At, ilo + 1, ihi, info, alpha1, beta1);
    if ((*info == 0) && ilascl) {
      notdone = true;
      while (notdone) {
        cfrom1 = anrmto * 2.0041683600089728E-292;
        cto1 = anrm / 4.9896007738368E+291;
        if ((cfrom1 > anrm) && (anrm != 0.0)) {
          mul = 2.0041683600089728E-292;
          anrmto = cfrom1;
        } else if (cto1 > anrmto) {
          mul = 4.9896007738368E+291;
          anrm = cto1;
        } else {
          mul = anrm / anrmto;
          notdone = false;
        }

        nzcount = alpha1->size[0];
        emxEnsureCapacity_creal_T(alpha1, nzcount);
        ii = alpha1->size[0];
        for (nzcount = 0; nzcount < ii; nzcount++) {
          alpha1->data[nzcount].re *= mul;
          alpha1->data[nzcount].im *= mul;
        }
      }
    }
  }

  emxFree_creal_T(&At);
}

/*
 * File trailer for xzgeev.c
 *
 * [EOF]
 */
