/** @file camera.h
 ** @brief Camera model and operations.
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

#ifndef _CAMERA_H_
#define _CAMERA_H_

/** @typedef struct Camera.
 ** @brief camera data structure.
 **/
typedef struct
{
	int xreso;			/*< horizontal resolution. */
	int yreso;			/*< vertical resolution. */
	float pp;			/*< pixel size. */
	float lens;			/*< focal length. */
	float sx;			/*< x-scale factor. */
	float sy;			/*< y-scale factor. */
	float fx;			/*< x-focal length. */
	float fy;			/*< y-focal length. */
	float cx;			/*< x-principal point. */
	float cy;			/*< y-principal point. */
	float C;			/*< constant. */
	float alpha;		/*< horizontal field of view. */
	float beta;			/*< vertical field of view. */
	float h;			/*< installation height. */
	float bevh;			/*< bird's eye view height. */
	float rx;			/*< rotation around x axis. */
	float ry;			/*< rotation around y axis. */
	float rz;			/*< rotation around z axis. */
	float A[12];		/*< 2D to 3D transformation matrix. */
	float R[16];		/*< rotation matrix. */
	float T[16];		/*< translation matrix. */
	float K[12];		/*< camera intrinsic matrix (3D to 2D). */
	float KTRA[9];		/*< perspective transform matrix. */
	float IKTRA[9];		/*< inverse perspective transform matrix. */
} Camera;

/** @name Camera model interface.
 ** @{ */
void CameraCreate(Camera *camera, int xreso, int yreso, int lens, float h,
                  float bevh, float rx, float ry, float rz);
void CalRotationAngleOfCam(Camera *camera, float yaw, float pitch,
                           float latoffset, float lanewidth);
void CalPerTransfMatOfCam(Camera *camera, float yaw, float pitch,
                          float latoffset, float lanewidth);
void CameraPerspectiveTransform(Camera *camera, unsigned char *src,
                                unsigned char *dst, int width, int height);
void CameraPerspectiveTransformPerPoint(Camera *camera, int sx, int sy,
                                        int *dx, int *dy, int flag);
/** @} */

#endif