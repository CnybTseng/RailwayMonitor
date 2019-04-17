/** @file scanlineparam.h
 ** @brief Scan line parameters calculator.
 ** @author Zhiwei Zeng
 ** @date 2018.09.11
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _SCANLINEPARAM_H_
#define _SCANLINEPARAM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rtwtypes.h"
#include "CalScanLineParam_types.h"
#include "rt_nonfinite.h"
#include "CalScanLineParam.h"
#include "CalScanLineParam_terminate.h"
#include "CalScanLineParam_initialize.h"

extern void CalScanLineParam_initialize(void);
extern void CalScanLineParam(float numinter, float mininter, float suminter,
                             float *maxinter, float *scale);
extern void CalScanLineParam_terminate(void);

#ifdef __cplusplus
}
#endif

#endif