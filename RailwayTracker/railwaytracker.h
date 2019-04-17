/** @file railwaytracker.h
 ** @brief Railway track base on extended kalman filter.
 ** @author Zhiwei Zeng
 ** @date 2018.07.23
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _RAILWAYTRACKER_H_
#define _RAILWAYTRACKER_H_

#include "camera.h"
#include "obstacledetector.h"
#include "pedestriandetector.h"

/** @typedef struct RailwayTracker.
 ** @brief Railway track data structure.
 **/
typedef struct tagRailwayTracker RailwayTracker;

/** @name Railway tracker interface.
 ** @{ */
RailwayTracker *RailwayTrackerCreate(Camera *camera, PedestrianDetector *pdetector,
                                     ObstacleDetector *odetector);
int RailwayTrackerPutData(RailwayTracker *self, unsigned char *data, int len);
int RailwayTrackerBegin(RailwayTracker *self);
int RailwayTrackerGetData(RailwayTracker *self, char *track, int len);
void RailwayTrackerOnOff(RailwayTracker *self, int onoff);
void RailwayTrackerEnd(RailwayTracker *self);
void RailwayTrackerDestroy(RailwayTracker *self);
/** @} */

#endif