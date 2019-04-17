/** @file list.h
 ** @brief Railway monitor list operation.
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

#ifndef _LIST_H_
#define _LIST_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <time.h>

#ifdef RAILWAYMONITOR_EXPORT
#	define RAILWAYMONITOR_API extern __declspec(dllexport) 
#elif defined(RAILWAYMONITOR_IMPORT)
#	define RAILWAYMONITOR_API extern "C"  __declspec(dllimport) 
#else
#	define RAILWAYMONITOR_API
#endif

#define RAILWAY_ALARM_TRACK_BROKEN			0x00000000
#define RAILWAY_ALARM_TRACK_OBSTACLE		0x00000001
#define RAILWAY_ALARM_TRACK_PEDESTRIAN		0x00000002

struct Track
{
	int px;				/*< 铁轨估计点X坐标. */
	int py;				/*< 铁轨估计点Y坐标. */
	int mx;				/*< 铁轨测量点X坐标,Y坐标=铁轨估计点Y坐标. */
	int pwx;			/*< 行人警戒线X坐标,Y坐标=铁轨估计点Y坐标. */
	int owx;			/*< 异物警戒线X坐标,Y坐标=铁轨估计点Y坐标. */
	int side;			/*< 左轨或右轨 */
	float dist;			/*< 前视距离. */
};

struct Alarm
{
	int type;				/*< 报警类型. */
	int top;				/*< 报警区域顶边. */
	int left;				/*< 报警区域左边. */
	int bottom;				/*< 报警区域底边. */
	int right;				/*< 报警区域右边. */
	int isdangerous;		/*< 报警对象是否超过警戒线. */
	float score;			/*< 报警得分(0~1). */
	float distance;			/*< 报警区域前视距离. */
	time_t time;			/*< 报警时间. */
};

struct Node
{
	void *val;				/*< 结点值. */
	struct Node *next;		/*< 下一个结点. */
};

/** @typedef struct List.
 ** @brief 列表的数据结构.
 **/
typedef struct
{
	struct Node *head;
	struct Node *tail;
	int size;
} List;

/** @name 列表操作接口.
 ** @{ */
RAILWAYMONITOR_API void ListInit(List *list);
RAILWAYMONITOR_API void *ListAlloc(size_t len);
RAILWAYMONITOR_API int ListAddTail(List *list, void *val);
RAILWAYMONITOR_API void ListConcatenate(List *first, List *second);
RAILWAYMONITOR_API void ListCopy(List *src, List *dst, size_t bpnv);
RAILWAYMONITOR_API struct Node *ListDelNode(List *list, struct Node *node);
RAILWAYMONITOR_API void ListDelAll(List *list);
/** @} */

#ifdef __cplusplus
}
#endif

#endif