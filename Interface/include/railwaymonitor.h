/** @file railwaymonitor.h
 ** @brief Railway monitor main module.
 ** @author Zhiwei Zeng
 ** @date 2018.08.14
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _RAILWAYMONITOR_H_
#define _RAILWAYMONITOR_H_

#include <windows.h>
#include "list.h"

#ifdef RAILWAYMONITOR_EXPORT
#	define RAILWAYMONITOR_API extern "C"  __declspec(dllexport) 
#elif defined(RAILWAYMONITOR_IMPORT)
#	define RAILWAYMONITOR_API extern "C"  __declspec(dllimport) 
#else
#	define RAILWAYMONITOR_API
#endif

/** @typedef struct RailwayMonitorOutput
 ** @brief 铁路监测输出数据结构
 **/
typedef struct
{
	List track;	/*< 铁轨坐标列表. */
	List alarm;			/*< 报警列表. */
} RailwayMonitorOutput;

/** @name Railway monitor interface.
 ** @{ */
RAILWAYMONITOR_API int RailwayMonitorCreate(int xreso, int yreso, int lens);
RAILWAYMONITOR_API int RailwayMonitorPutData(unsigned char *data, int len);
RAILWAYMONITOR_API int RailwayMonitorBegin();
RAILWAYMONITOR_API int RailwayMonitorGetData(RailwayMonitorOutput *output);
RAILWAYMONITOR_API void RailwayMonitorEnd();
RAILWAYMONITOR_API void RailwayMonitorDestroy();
/** @} */ 

#endif