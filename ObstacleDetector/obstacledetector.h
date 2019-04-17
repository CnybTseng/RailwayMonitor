/** @file obstacledetector.h
 ** @brief Railway obstacle detection base on computer vision.
 ** @author Zhiwei Zeng
 ** @date 2018.08.01
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the railway monitor toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#ifndef _OBSTACLEDETECTOR_H_
#define _OBSTACLEDETECTOR_H_

#include "camera.h"

/** @typedef struct ObstacleDetector
 ** @brief Railway obstacle detector data structure.
 **/
typedef struct tagObstacleDetector ObstacleDetector;

/** @name Railway obstacle detector interface.
 ** @{ */
ObstacleDetector *ObstacleDetectorCreate(Camera *camera);
int ObstacleDetectorPutImage(ObstacleDetector *self, unsigned char *data, int len);
int ObstacleDetectorPutTrack(ObstacleDetector *self, char *data, int len);
int ObstacleDetectorBegin(ObstacleDetector *self);
int ObstacleDetectorGetData(ObstacleDetector *self, char *alarm, int len);
void ObstacleDetectorOnOff(ObstacleDetector *self, int onoff);
void ObstacleDetectorEnd(ObstacleDetector *self);
void ObstacleDetectorDestroy(ObstacleDetector *self);
/** @} */

#endif