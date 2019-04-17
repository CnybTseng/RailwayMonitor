/*
 * File: xzhseqr.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "xzhseqr.h"
#include "xscal.h"
#include "xzlarfg.h"
#include "sqrt.h"
#include "CalScanLineParam_rtwutil.h"

/* Function Definitions */

/*
 * Arguments    : emxArray_creal_T *h
 * Return Type  : int
 */
int eml_zlahqr(emxArray_creal_T *h)
{
  int info;
  int n;
  int u1;
  double itmax;
  int ldh;
  int i;
  double SMLNUM;
  double tst;
  boolean_T exitg1;
  double aa;
  double ba;
  int L;
  creal_T u2;
  boolean_T goto140;
  int its;
  boolean_T exitg2;
  int k;
  boolean_T exitg3;
  double htmp1;
  creal_T y;
  double ab;
  boolean_T goto70;
  int m;
  double x_re;
  double u_re;
  double x_im;
  double u_im;
  double s;
  int b_k;
  creal_T v[2];
  double b_SMLNUM;
  int i2;
  n = h->size[0];
  u1 = h->size[0];
  if (10 > u1) {
    u1 = 10;
  }

  itmax = 30.0 * (double)u1;
  ldh = h->size[0];
  info = 0;
  if (1 != h->size[0]) {
    for (u1 = 0; u1 < n - 3; u1++) {
      h->data[(u1 + h->size[0] * u1) + 2].re = 0.0;
      h->data[(u1 + h->size[0] * u1) + 2].im = 0.0;
      h->data[(u1 + h->size[0] * u1) + 3].re = 0.0;
      h->data[(u1 + h->size[0] * u1) + 3].im = 0.0;
    }

    if (1 <= n - 2) {
      h->data[(n + h->size[0] * (n - 3)) - 1].re = 0.0;
      h->data[(n + h->size[0] * (n - 3)) - 1].im = 0.0;
    }

    for (i = 1; i < n; i++) {
      if (h->data[i + h->size[0] * (i - 1)].im != 0.0) {
        tst = h->data[i + h->size[0] * (i - 1)].re;
        aa = h->data[i + h->size[0] * (i - 1)].im;
        ba = fabs(h->data[i + h->size[0] * (i - 1)].re) + fabs(h->data[i +
          h->size[0] * (i - 1)].im);
        if (aa == 0.0) {
          u2.re = tst / ba;
          u2.im = 0.0;
        } else if (tst == 0.0) {
          u2.re = 0.0;
          u2.im = aa / ba;
        } else {
          u2.re = tst / ba;
          u2.im = aa / ba;
        }

        ba = rt_hypotd_snf(u2.re, u2.im);
        if (-u2.im == 0.0) {
          u2.re /= ba;
          u2.im = 0.0;
        } else if (u2.re == 0.0) {
          u2.re = 0.0;
          u2.im = -u2.im / ba;
        } else {
          u2.re /= ba;
          u2.im = -u2.im / ba;
        }

        tst = h->data[i + h->size[0] * (i - 1)].re;
        htmp1 = h->data[i + h->size[0] * (i - 1)].im;
        h->data[i + h->size[0] * (i - 1)].re = rt_hypotd_snf(tst, htmp1);
        h->data[i + h->size[0] * (i - 1)].im = 0.0;
        b_xscal(n - i, u2, h, (i + i * ldh) + 1, ldh);
        y.re = u2.re;
        y.im = -u2.im;
        u1 = i + 2;
        if (n < u1) {
          u1 = n;
        }

        xscal(u1, y, h, 1 + i * ldh);
      }
    }

    SMLNUM = 2.2250738585072014E-308 * ((double)n / 2.2204460492503131E-16);
    i = n - 1;
    exitg1 = false;
    while ((!exitg1) && (i + 1 >= 1)) {
      L = -1;
      goto140 = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its <= (int)itmax)) {
        k = i;
        exitg3 = false;
        while ((!exitg3) && ((k + 1 > L + 2) && (!(fabs(h->data[k + h->size[0] *
                   (k - 1)].re) + fabs(h->data[k + h->size[0] * (k - 1)].im) <=
                  SMLNUM)))) {
          tst = (fabs(h->data[(k + h->size[0] * (k - 1)) - 1].re) + fabs(h->
                  data[(k + h->size[0] * (k - 1)) - 1].im)) + (fabs(h->data[k +
            h->size[0] * k].re) + fabs(h->data[k + h->size[0] * k].im));
          if (tst == 0.0) {
            if (k - 1 >= 1) {
              tst = fabs(h->data[(k + h->size[0] * (k - 2)) - 1].re);
            }

            if (k + 2 <= n) {
              tst += fabs(h->data[(k + h->size[0] * k) + 1].re);
            }
          }

          if (fabs(h->data[k + h->size[0] * (k - 1)].re) <=
              2.2204460492503131E-16 * tst) {
            htmp1 = fabs(h->data[k + h->size[0] * (k - 1)].re) + fabs(h->data[k
              + h->size[0] * (k - 1)].im);
            tst = fabs(h->data[(k + h->size[0] * k) - 1].re) + fabs(h->data[(k +
              h->size[0] * k) - 1].im);
            if (htmp1 > tst) {
              ab = htmp1;
              ba = tst;
            } else {
              ab = tst;
              ba = htmp1;
            }

            htmp1 = fabs(h->data[k + h->size[0] * k].re) + fabs(h->data[k +
              h->size[0] * k].im);
            x_re = h->data[(k + h->size[0] * (k - 1)) - 1].re - h->data[k +
              h->size[0] * k].re;
            x_im = h->data[(k + h->size[0] * (k - 1)) - 1].im - h->data[k +
              h->size[0] * k].im;
            tst = fabs(x_re) + fabs(x_im);
            if (htmp1 > tst) {
              aa = htmp1;
              htmp1 = tst;
            } else {
              aa = tst;
            }

            s = aa + ab;
            tst = 2.2204460492503131E-16 * (htmp1 * (aa / s));
            if ((SMLNUM > tst) || rtIsNaN(tst)) {
              b_SMLNUM = SMLNUM;
            } else {
              b_SMLNUM = tst;
            }

            if (ba * (ab / s) <= b_SMLNUM) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }

        L = k - 1;
        if (k + 1 > 1) {
          h->data[k + h->size[0] * (k - 1)].re = 0.0;
          h->data[k + h->size[0] * (k - 1)].im = 0.0;
        }

        if (k + 1 >= i + 1) {
          goto140 = true;
          exitg2 = true;
        } else {
          if (its == 10) {
            ba = 0.75 * fabs(h->data[(k + h->size[0] * k) + 1].re) + h->data[k +
              h->size[0] * k].re;
            ab = h->data[k + h->size[0] * k].im;
          } else if (its == 20) {
            ba = 0.75 * fabs(h->data[i + h->size[0] * (i - 1)].re) + h->data[i +
              h->size[0] * i].re;
            ab = h->data[i + h->size[0] * i].im;
          } else {
            ba = h->data[i + h->size[0] * i].re;
            ab = h->data[i + h->size[0] * i].im;
            y = h->data[(i + h->size[0] * i) - 1];
            b_sqrt(&y);
            u2 = h->data[i + h->size[0] * (i - 1)];
            b_sqrt(&u2);
            u_re = y.re * u2.re - y.im * u2.im;
            u_im = y.re * u2.im + y.im * u2.re;
            s = fabs(u_re) + fabs(u_im);
            if (s != 0.0) {
              tst = h->data[(i + h->size[0] * (i - 1)) - 1].re - h->data[i +
                h->size[0] * i].re;
              htmp1 = h->data[(i + h->size[0] * (i - 1)) - 1].im - h->data[i +
                h->size[0] * i].im;
              x_re = 0.5 * tst;
              x_im = 0.5 * htmp1;
              aa = fabs(x_re) + fabs(x_im);
              tst = fabs(x_re) + fabs(x_im);
              if (!((s > tst) || rtIsNaN(tst))) {
                s = tst;
              }

              if (x_im == 0.0) {
                ba = x_re / s;
                ab = 0.0;
              } else if (x_re == 0.0) {
                ba = 0.0;
                ab = x_im / s;
              } else {
                ba = x_re / s;
                ab = x_im / s;
              }

              tst = ba;
              ba = ba * ba - ab * ab;
              ab = tst * ab + ab * tst;
              if (u_im == 0.0) {
                u2.re = u_re / s;
                u2.im = 0.0;
              } else if (u_re == 0.0) {
                u2.re = 0.0;
                u2.im = u_im / s;
              } else {
                u2.re = u_re / s;
                u2.im = u_im / s;
              }

              y.re = ba + (u2.re * u2.re - u2.im * u2.im);
              y.im = ab + (u2.re * u2.im + u2.im * u2.re);
              b_sqrt(&y);
              y.re *= s;
              y.im *= s;
              if (aa > 0.0) {
                if (x_im == 0.0) {
                  ba = x_re / aa;
                  ab = 0.0;
                } else if (x_re == 0.0) {
                  ba = 0.0;
                  ab = x_im / aa;
                } else {
                  ba = x_re / aa;
                  ab = x_im / aa;
                }

                if (ba * y.re + ab * y.im < 0.0) {
                  y.re = -y.re;
                  y.im = -y.im;
                }
              }

              ba = x_re + y.re;
              htmp1 = x_im + y.im;
              if (htmp1 == 0.0) {
                if (u_im == 0.0) {
                  x_re = u_re / ba;
                  tst = 0.0;
                } else if (u_re == 0.0) {
                  x_re = 0.0;
                  tst = u_im / ba;
                } else {
                  x_re = u_re / ba;
                  tst = u_im / ba;
                }
              } else if (ba == 0.0) {
                if (u_re == 0.0) {
                  x_re = u_im / htmp1;
                  tst = 0.0;
                } else if (u_im == 0.0) {
                  x_re = 0.0;
                  tst = -(u_re / htmp1);
                } else {
                  x_re = u_im / htmp1;
                  tst = -(u_re / htmp1);
                }
              } else {
                ab = fabs(ba);
                tst = fabs(htmp1);
                if (ab > tst) {
                  s = htmp1 / ba;
                  tst = ba + s * htmp1;
                  x_re = (u_re + s * u_im) / tst;
                  tst = (u_im - s * u_re) / tst;
                } else if (tst == ab) {
                  if (ba > 0.0) {
                    aa = 0.5;
                  } else {
                    aa = -0.5;
                  }

                  if (htmp1 > 0.0) {
                    tst = 0.5;
                  } else {
                    tst = -0.5;
                  }

                  x_re = (u_re * aa + u_im * tst) / ab;
                  tst = (u_im * aa - u_re * tst) / ab;
                } else {
                  s = ba / htmp1;
                  tst = htmp1 + s * ba;
                  x_re = (s * u_re + u_im) / tst;
                  tst = (s * u_im - u_re) / tst;
                }
              }

              ba = h->data[i + h->size[0] * i].re - (u_re * x_re - u_im * tst);
              ab = h->data[i + h->size[0] * i].im - (u_re * tst + u_im * x_re);
            }
          }

          goto70 = false;
          m = i;
          exitg3 = false;
          while ((!exitg3) && (m > k + 1)) {
            u2.re = h->data[(m + h->size[0] * (m - 1)) - 1].re - ba;
            u2.im = h->data[(m + h->size[0] * (m - 1)) - 1].im - ab;
            tst = h->data[m + h->size[0] * (m - 1)].re;
            s = (fabs(u2.re) + fabs(u2.im)) + fabs(tst);
            if (u2.im == 0.0) {
              u2.re /= s;
              u2.im = 0.0;
            } else if (u2.re == 0.0) {
              u2.re = 0.0;
              u2.im /= s;
            } else {
              u2.re /= s;
              u2.im /= s;
            }

            tst /= s;
            v[0] = u2;
            v[1].re = tst;
            v[1].im = 0.0;
            if (fabs(h->data[(m + h->size[0] * (m - 2)) - 1].re) * fabs(tst) <=
                2.2204460492503131E-16 * ((fabs(u2.re) + fabs(u2.im)) * ((fabs
                   (h->data[(m + h->size[0] * (m - 1)) - 1].re) + fabs(h->data
                    [(m + h->size[0] * (m - 1)) - 1].im)) + (fabs(h->data[m +
                    h->size[0] * m].re) + fabs(h->data[m + h->size[0] * m].im)))))
            {
              goto70 = true;
              exitg3 = true;
            } else {
              m--;
            }
          }

          if (!goto70) {
            u2.re = h->data[k + h->size[0] * k].re - ba;
            u2.im = h->data[k + h->size[0] * k].im - ab;
            tst = h->data[(k + h->size[0] * k) + 1].re;
            s = (fabs(u2.re) + fabs(u2.im)) + fabs(tst);
            if (u2.im == 0.0) {
              u2.re /= s;
              u2.im = 0.0;
            } else if (u2.re == 0.0) {
              u2.re = 0.0;
              u2.im /= s;
            } else {
              u2.re /= s;
              u2.im /= s;
            }

            tst /= s;
            v[0] = u2;
            v[1].re = tst;
            v[1].im = 0.0;
          }

          for (b_k = m; b_k <= i; b_k++) {
            if (b_k > m) {
              v[0] = h->data[(b_k + h->size[0] * (b_k - 2)) - 1];
              v[1] = h->data[b_k + h->size[0] * (b_k - 2)];
            }

            u2 = xzlarfg(&v[0], &v[1]);
            if (b_k > m) {
              h->data[(b_k + h->size[0] * (b_k - 2)) - 1] = v[0];
              h->data[b_k + h->size[0] * (b_k - 2)].re = 0.0;
              h->data[b_k + h->size[0] * (b_k - 2)].im = 0.0;
            }

            htmp1 = u2.re * v[1].re - u2.im * v[1].im;
            for (u1 = b_k - 1; u1 < n; u1++) {
              tst = u2.re * h->data[(b_k + h->size[0] * u1) - 1].re - -u2.im *
                h->data[(b_k + h->size[0] * u1) - 1].im;
              aa = u2.re * h->data[(b_k + h->size[0] * u1) - 1].im + -u2.im *
                h->data[(b_k + h->size[0] * u1) - 1].re;
              ba = tst + htmp1 * h->data[b_k + h->size[0] * u1].re;
              ab = aa + htmp1 * h->data[b_k + h->size[0] * u1].im;
              h->data[(b_k + h->size[0] * u1) - 1].re -= ba;
              h->data[(b_k + h->size[0] * u1) - 1].im -= ab;
              h->data[b_k + h->size[0] * u1].re -= ba * v[1].re - ab * v[1].im;
              h->data[b_k + h->size[0] * u1].im -= ba * v[1].im + ab * v[1].re;
            }

            if (b_k + 2 < i + 1) {
              i2 = b_k;
            } else {
              i2 = i - 1;
            }

            for (u1 = 0; u1 < i2 + 2; u1++) {
              tst = u2.re * h->data[u1 + h->size[0] * (b_k - 1)].re - u2.im *
                h->data[u1 + h->size[0] * (b_k - 1)].im;
              aa = u2.re * h->data[u1 + h->size[0] * (b_k - 1)].im + u2.im *
                h->data[u1 + h->size[0] * (b_k - 1)].re;
              ba = tst + htmp1 * h->data[u1 + h->size[0] * b_k].re;
              ab = aa + htmp1 * h->data[u1 + h->size[0] * b_k].im;
              h->data[u1 + h->size[0] * (b_k - 1)].re -= ba;
              h->data[u1 + h->size[0] * (b_k - 1)].im -= ab;
              h->data[u1 + h->size[0] * b_k].re -= ba * v[1].re - ab * -v[1].im;
              h->data[u1 + h->size[0] * b_k].im -= ba * -v[1].im + ab * v[1].re;
            }

            if ((b_k == m) && (m > k + 1)) {
              u2.re = 1.0 - u2.re;
              u2.im = 0.0 - u2.im;
              ba = rt_hypotd_snf(u2.re, u2.im);
              if (u2.im == 0.0) {
                u2.re /= ba;
                u2.im = 0.0;
              } else if (u2.re == 0.0) {
                u2.re = 0.0;
                u2.im /= ba;
              } else {
                u2.re /= ba;
                u2.im /= ba;
              }

              tst = h->data[m + h->size[0] * (m - 1)].re;
              htmp1 = h->data[m + h->size[0] * (m - 1)].im;
              h->data[m + h->size[0] * (m - 1)].re = tst * u2.re - htmp1 *
                -u2.im;
              h->data[m + h->size[0] * (m - 1)].im = tst * -u2.im + htmp1 *
                u2.re;
              if (m + 2 <= i + 1) {
                tst = h->data[(m + h->size[0] * m) + 1].re;
                htmp1 = h->data[(m + h->size[0] * m) + 1].im;
                h->data[(m + h->size[0] * m) + 1].re = tst * u2.re - htmp1 *
                  u2.im;
                h->data[(m + h->size[0] * m) + 1].im = tst * u2.im + htmp1 *
                  u2.re;
              }

              for (u1 = m; u1 <= i + 1; u1++) {
                if (u1 != m + 1) {
                  if (n > u1) {
                    b_xscal(n - u1, u2, h, u1 + u1 * ldh, ldh);
                  }

                  y.re = u2.re;
                  y.im = -u2.im;
                  xscal(u1 - 1, y, h, 1 + (u1 - 1) * ldh);
                }
              }
            }
          }

          u2 = h->data[i + h->size[0] * (i - 1)];
          if (h->data[i + h->size[0] * (i - 1)].im != 0.0) {
            tst = rt_hypotd_snf(h->data[i + h->size[0] * (i - 1)].re, h->data[i
                                + h->size[0] * (i - 1)].im);
            h->data[i + h->size[0] * (i - 1)].re = tst;
            h->data[i + h->size[0] * (i - 1)].im = 0.0;
            if (u2.im == 0.0) {
              u2.re /= tst;
              u2.im = 0.0;
            } else if (u2.re == 0.0) {
              u2.re = 0.0;
              u2.im /= tst;
            } else {
              u2.re /= tst;
              u2.im /= tst;
            }

            if (n > i + 1) {
              y.re = u2.re;
              y.im = -u2.im;
              b_xscal((n - i) - 1, y, h, (i + (i + 1) * ldh) + 1, ldh);
            }

            xscal(i, u2, h, 1 + i * ldh);
          }

          its++;
        }
      }

      if (!goto140) {
        info = i + 1;
        exitg1 = true;
      } else {
        i = L;
      }
    }
  }

  return info;
}

/*
 * File trailer for xzhseqr.c
 *
 * [EOF]
 */
