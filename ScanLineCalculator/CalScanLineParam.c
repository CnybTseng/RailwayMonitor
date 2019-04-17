/*
 * File: CalScanLineParam.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

/* Include Files */
#include <math.h>
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "CalScanLineParam_emxutil.h"
#include "roots.h"

/* Function Declarations */
static float rt_powf_snf(float u0, float u1);

/* Function Definitions */

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_powf_snf(float u0, float u1)
{
  float y;
  float f0;
  float f1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f0 = (float)fabs(u0);
    f1 = (float)fabs(u1);
    if (rtIsInfF(u1)) {
      if (f0 == 1.0F) {
        y = 1.0F;
      } else if (f0 > 1.0F) {
        if (u1 > 0.0F) {
          y = ((real32_T)rtInf);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = ((real32_T)rtInf);
      }
    } else if (f1 == 0.0F) {
      y = 1.0F;
    } else if (f1 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (float)sqrt(u0);
    } else if ((u0 < 0.0F) && (u1 > (float)floor(u1))) {
      y = ((real32_T)rtNaN);
    } else {
      y = (float)pow(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : float numinter
 *                float mininter
 *                float suminter
 *                float *maxinter
 *                float *scale
 * Return Type  : void
 */
void CalScanLineParam(float numinter, float mininter, float suminter, float
                      *maxinter, float *scale)
{
  emxArray_real_T *P;
  int i;
  int loop_ub;
  emxArray_creal_T *rs;
  double b_scale;
  boolean_T exitg1;
  emxInit_real_T(&P, 2);
  i = P->size[0] * P->size[1];
  P->size[0] = 1;
  P->size[1] = (int)(numinter + 1.0F);
  emxEnsureCapacity_real_T(P, i);
  loop_ub = (int)(numinter + 1.0F);
  for (i = 0; i < loop_ub; i++) {
    P->data[i] = 0.0;
  }

  emxInit_creal_T(&rs, 1);
  P->data[0] = suminter - mininter;
  P->data[1] = -suminter;
  P->data[(int)(numinter + 1.0F) - 1] = mininter;
  roots(P, rs);

  /*  Get real solution */
  b_scale = 1.0;
  i = 0;
  emxFree_real_T(&P);
  exitg1 = false;
  while ((!exitg1) && (i <= rs->size[0] - 1)) {
    if ((rs->data[i].re < 0.99999) && (fabs(rs->data[i].im) < 1.0E-10)) {
      b_scale = rs->data[i].re;
      exitg1 = true;
    } else {
      i++;
    }
  }

  emxFree_creal_T(&rs);
  *scale = (float)b_scale;
  *maxinter = suminter * (1.0F - (float)b_scale) / (1.0F - rt_powf_snf((float)
    b_scale, numinter));
}

/*
 * File trailer for CalScanLineParam.c
 *
 * [EOF]
 */
