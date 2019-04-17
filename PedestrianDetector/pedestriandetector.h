/** @file pedestriandetector.h
 ** @brief Railway pedestrian detection base on HOG and SVM.
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

#ifndef _PEDESTRIANDETECTOR_H_
#define _PEDESTRIANDETECTOR_H_

#include "camera.h"

/** @typedef struct PedestrianDetector
 ** @brief pedestrian detector data structure.
 **/
typedef struct tagPedestrianDetector PedestrianDetector;

/** @name Pedestrian detector interface.
 ** @{ */
PedestrianDetector *PedestrianDetectorCreate(Camera *camera);
int PedestrianDetectorPutImage(PedestrianDetector *self, unsigned char *data, int len);
int PedestrianDetectorPutTrack(PedestrianDetector *self, char *data, int len);
int PedestrianDetectorBegin(PedestrianDetector *self);
int PedestrianDetectorGetData(PedestrianDetector *self, char *alarm, int len);
void PedestrianDetectorOnOff(PedestrianDetector *self, int onoff);
void PedestrianDetectorEnd(PedestrianDetector *self);
void PedestrianDetectorDestroy(PedestrianDetector *self);
/** @} */

#endif