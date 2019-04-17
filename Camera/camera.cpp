/** @file camera.cpp - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "opencv2/opencv.hpp"
#include "camera.h"

/** @name Local functions.
 ** @{ */
static void SetAMatrix(Camera *camera);
static void SetRMatrix(Camera *camera);
static void SetTMatrix(Camera *camera);
static void SetKMatrix(Camera *camera);
/** @} */

/** @brief Create a camera model.
 ** @param camera camera instance.
 ** @param xreso x-resolution.
 ** @param yreso y-resolution.
 ** @param lens focal lens.
 ** @param h install height.
 ** @param bevh bird's eye view height.
 ** @param rx rotation around x axis.
 ** @param ry rotation around y axis.
 ** @param rz rotation around z axis.
 **/
void CameraCreate(Camera *camera, int xreso, int yreso, int lens, float h,
                  float bevh, float rx, float ry, float rz)
{
	camera->xreso = xreso;
	camera->yreso = yreso;
	camera->pp = 17.0f;
	camera->lens = (float)lens;
	camera->sx = 1000 / camera->pp;
	camera->sy = camera->sx;
	camera->fx = camera->sx * camera->lens;
	camera->fy = camera->fx;
	camera->cx = camera->xreso / 2.0f;
	camera->cy = camera->yreso / 2.0f;
	camera->C = 17.45f;
	camera->alpha = camera->pp * camera->xreso / camera->lens / camera->C;;
	camera->beta = camera->pp * camera->yreso / camera->lens / camera->C;
	camera->h = h;
	camera->bevh = bevh;
	camera->rx = rx;
	camera->ry = ry;
	camera->rz = rz;
	SetAMatrix(camera);
	SetRMatrix(camera);
	SetTMatrix(camera);
	SetKMatrix(camera);
	
	cv::Mat _A(4, 3, CV_32FC1, camera->A);
	cv::Mat _R(4, 4, CV_32FC1, camera->R);
	cv::Mat _T(4, 4, CV_32FC1, camera->T);
	cv::Mat _K(3, 4, CV_32FC1, camera->K);
	cv::Mat _KTRA(3, 3, CV_32FC1, camera->KTRA);
	cv::Mat _IKTRA(3, 3, CV_32FC1, camera->IKTRA);
	
	_KTRA = _K * (_T * (_R * _A));
	cv::invert(_KTRA, _IKTRA);
}

/** @brief Calculate rotation angle of camera.
 **        Only yaw angle, pitch angle, lateral offset, and lane width of
 **        railway state model are considerate.
 ** @param camera camera model.
 ** @param yaw yaw angle.
 ** @param pitch pitch angle.
 ** @param latoffset lateral offset.
 ** @param lanewidth lane width.
 **/
void CalRotationAngleOfCam(Camera *camera, float yaw, float pitch,
                           float latoffset, float lanewidth)
{
	// Calculate quadrant points.
	int X[4] = {0, 0, 0, 0};
	int Y[4] = {camera->yreso - 1, camera->yreso - 1, 0, 0};
	int sign[4] = {-1, 1, -1, 1};
	
	for (int i = 0; i < 4; i++) {
		double dist = camera->fy * camera->h / ((double)Y[i] - camera->fy * pitch - camera->cy);
		X[i] = (int)(camera->fx * (sign[i] * lanewidth / 2 - latoffset + yaw * dist) / dist + camera->cx);
	}
	
	int bevX[4];
	int bevY[4];
	Camera candidate;
	const float hPi = 1.57079633f;
	
	for (float rx = -hPi; rx < 0; rx += 0.0001f) {
		CameraCreate(&candidate, camera->xreso, camera->yreso, (int)camera->lens, camera->h,
			camera->bevh, rx, 0, 0);
		
		int outofrange = 0;
		for (int i = 0; i < 4; i++) {
			CameraPerspectiveTransformPerPoint(&candidate, X[i], Y[i], &bevX[i], &bevY[i], 1);
		}
		
		if (bevX[0] == bevX[1] || bevX[2] == bevX[3] ||
			bevY[0] == bevY[2] || bevY[1] == bevY[3]) {
			continue;
		}
		
		// Check if the two lines are parallel with each other.
		if (bevX[0] == bevX[2] && bevX[1] == bevX[3]) {
			printf("camera rotation angle=%f<two vertical lines>.\n", rx);
			*camera = candidate;
			break;
		}
				
		float k1 = (bevY[2] - bevY[0]) / ((float)bevX[2] - bevX[0]);
		float k2 = (bevY[3] - bevY[1]) / ((float)bevX[3] - bevX[1]);
		if (fabs(k1 - k2) < 0.001f) {
			printf("camera rotation angle=%f.\n", rx);
			printf("%d %d %d %d %d %d %d %d.\n", bevX[0], bevX[1], bevX[2], bevX[3], bevY[0], bevY[1],
				bevY[2], bevY[3]);
			*camera = candidate;
			break;
		}
	}

	// Debug.
	cv::Mat BW(camera->yreso, camera->xreso, CV_8UC3, cv::Scalar(0));
	cv::line(BW, cv::Point(X[0], Y[0]), cv::Point(X[2], Y[2]), cv::Scalar(0, 255, 255));
	cv::line(BW, cv::Point(X[1], Y[1]), cv::Point(X[3], Y[3]), cv::Scalar(0, 255, 255));
	cv::line(BW, cv::Point(bevX[0], bevY[0]), cv::Point(bevX[2], bevY[2]), cv::Scalar(255, 255, 255));
	cv::line(BW, cv::Point(bevX[1], bevY[1]), cv::Point(bevX[3], bevY[3]), cv::Scalar(255, 255, 255));
	cv::imwrite("idealTrack.png", BW);
}

/** @brief Calculate perspective transform matrix from quadrant points.
 **        Only yaw angle, pitch angle, lateral offset, and lane width of
 **        railway state model are considerate.
 ** @param camera camera model.
 ** @param yaw yaw angle.
 ** @param pitch pitch angle.
 ** @param latoffset lateral offset.
 ** @param lanewidth lane width.
 **/ 
void CalPerTransfMatOfCam(Camera *camera, float yaw, float pitch,
                          float latoffset, float lanewidth)
{
	// Calculate source quadrant points.
	cv::Point2f src[4] = {cv::Point2f(0, camera->yreso - 1.0f), cv::Point2f(0, camera->yreso - 1.0f),
		cv::Point2f(0, 0), cv::Point2f(0, 0)};
	int sign[4] = {-1, 1, -1, 1};
	
	for (int i = 0; i < 4; i++) {
		double dist = camera->fy * camera->h / ((double)src[i].y - camera->fy * pitch - camera->cy + 1e-20);
		src[i].x = (float)(camera->fx * (sign[i] * lanewidth / 2 - latoffset + yaw * dist) / dist + camera->cx);
	}
	
	// The corresponding perspective transformed points.
	float left = (src[0].x + src[2].x) / 2;
	float right = (src[1].x + src[3].x) / 2;
	cv::Point2f dst[4] = {cv::Point2f(left, src[0].y), cv::Point2f(right, src[1].y),
		cv::Point2f(left, src[2].y), cv::Point2f(right, src[3].y)};
	
	// Calculate inverse perspective transform matrix.
	cv::Mat KTRA = cv::getPerspectiveTransform(dst, src);
	
	cv::Mat _KTRA(3, 3, CV_32FC1, camera->KTRA);	
	cv::Mat _IKTRA(3, 3, CV_32FC1, camera->IKTRA);
	
	KTRA.convertTo(_KTRA, CV_32FC1);
	cv::invert(_KTRA, _IKTRA);
	
	// Debug.
	cv::Mat BW(camera->yreso, camera->xreso, CV_8UC3, cv::Scalar(0));
	cv::line(BW, src[0], src[2], cv::Scalar(0, 255, 255));
	cv::line(BW, src[1], src[3], cv::Scalar(0, 255, 255));
	cv::line(BW, dst[0], dst[2], cv::Scalar(255, 255, 255));
	cv::line(BW, dst[1], dst[3], cv::Scalar(255, 255, 255));
	cv::imwrite("idealTrack.png", BW);
}

/** @brief Calculate bird's eye view of a image.
 ** @param camera camera model.
 ** @param src source image.
 ** @param dst destination image.
 ** @param width image width.
 ** @param height image height.
 **/
void CameraPerspectiveTransform(Camera *camera, unsigned char *src,
                                unsigned char *dst, int width, int height)
{
	cv::Mat _src(height, width, CV_16UC1, src);
	cv::Mat _dst(height, width, CV_16UC1, dst);
	cv::Mat KTRA(3, 3, CV_32FC1, camera->KTRA);
	
	cv::warpPerspective(_src, _dst, KTRA, cv::Size(width, height),
		cv::INTER_CUBIC | cv::WARP_INVERSE_MAP, cv::BORDER_REPLICATE);
}

/** @brief Set elements of 2D->3D transform matrix.
 ** @param camera camera model.
 **/
void SetAMatrix(Camera *camera)
{
	camera->A[0]  = 1;
	camera->A[1]  = 0;
	camera->A[2]  = -camera->xreso / 2.0f;
	
	camera->A[3]  = 0;
	camera->A[4]  = 1;
	camera->A[5]  = -camera->yreso / 2.0f;
	
	camera->A[6]  = 0;
	camera->A[7]  = 0;
	camera->A[8]  = 0;
	
	camera->A[9]  = 0;
	camera->A[10] = 0;
	camera->A[11] = 1;
}

/** @brief Set elements of rotation transform matrix.
 ** @param camera camera model.
 **/
void SetRMatrix(Camera *camera)
{
	float rx = camera->rx;
	float ry = camera->ry;
	float rz = camera->rz;
	
	float RX[16] = {
		1,       0,        0, 0,
		0, cos(rx), -sin(rx), 0,
		0, sin(rx),  cos(rx), 0,
		0,       0,        0, 1
	};
	
	float RY[16] = {
		cos(ry), 0, -sin(ry), 0,
		      0, 1,        0, 0,
		sin(ry), 0,  cos(ry), 0,
		      0, 0,        0, 1
	};
	
	float RZ[16] = {
		cos(rz), -sin(rz), 0, 0,
		sin(rz),  cos(rz), 0, 0,
		      0,        0, 1, 0,
			  0,        0, 0, 1
	};
	
	cv::Mat _RX(4, 4, CV_32FC1, RX);
	cv::Mat _RY(4, 4, CV_32FC1, RY);
	cv::Mat _RZ(4, 4, CV_32FC1, RZ);
	cv::Mat  _R(4, 4, CV_32FC1, camera->R);
	
	_R = _RX * _RY * _RZ;
}

/** @brief Set elements of translation transform matrix.
 ** @param camera camera model.
 **/
void SetTMatrix(Camera *camera)
{
	camera->T[0]  = 1;
	camera->T[1]  = 0;
	camera->T[2]  = 0;
	camera->T[3]  = 0;
	
	camera->T[4]  = 0;
	camera->T[5]  = 1;
	camera->T[6]  = 0;
	camera->T[7]  = 0;
	
	camera->T[8]  = 0;
	camera->T[9]  = 0;
	camera->T[10] = 1;
	camera->T[11] = -camera->bevh / (sin(camera->rx) + 1e-7f);
	
	camera->T[12] = 0;
	camera->T[13] = 0;
	camera->T[14] = 0;
	camera->T[15] = 1;
}

/** @brief Set elements camera intrinsic matrix.
 ** @param camera camera model.
 **/
void SetKMatrix(Camera *camera)
{
	camera->K[0]  = camera->lens;
	camera->K[1]  = 0;
	camera->K[2]  = camera->xreso / 2.0f;
	camera->K[3]  = 0;
	              
	camera->K[4]  = 0;
	camera->K[5]  = camera->lens;
	camera->K[6]  = camera->yreso / 2.0f;
	camera->K[7]  = 0;
	              
	camera->K[8]  = 0;
	camera->K[9]  = 0;
	camera->K[10] = 1;
	camera->K[11] = 0;
}

/** @brief Perspective transform for single point.
 ** @param camera camera model.
 ** @param sx source x position.
 ** @param sy source y position.
 ** @param dx destination x position.
 ** @param dy destination y position.
 ** @param flag transform direction.
 **/
void CameraPerspectiveTransformPerPoint(Camera *camera, int sx, int sy,
                                        int *dx, int *dy, int flag)
{
	float *ktra = NULL;
	if (0 == flag) {
		ktra = camera->KTRA;
	} else {
		ktra = camera->IKTRA;
	}
	
	float denominator = ktra[6] * sx + ktra[7] * sy + ktra[8];
	*dx = (int)((ktra[0] * sx + ktra[1] * sy + ktra[2]) / denominator + 0.5f);
	*dy = (int)((ktra[3] * sx + ktra[4] * sy + ktra[5]) / denominator + 0.5f);
}