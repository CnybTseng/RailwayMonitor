/*
 * File: CalScanLineParam_emxutil.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 11-Sep-2018 11:05:44
 */

#ifndef CALSCANLINEPARAM_EMXUTIL_H
#define CALSCANLINEPARAM_EMXUTIL_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "CalScanLineParam_types.h"

/* Function Declarations */
extern void emxEnsureCapacity_creal_T(emxArray_creal_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_creal_T1(emxArray_creal_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxFree_creal_T(emxArray_creal_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_creal_T(emxArray_creal_T **pEmxArray, int numDimensions);
extern void emxInit_creal_T1(emxArray_creal_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

/*
 * File trailer for CalScanLineParam_emxutil.h
 *
 * [EOF]
 */
