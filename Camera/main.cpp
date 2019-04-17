/** @file main.cpp
 ** @brief Test perspective transform.
 ** @author Zhiwei Zeng
 ** @date 2018.08.02
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

void Help();
void test();

int main(int argc, char *argv[])
{
	if (argc <= 1) {
		Help();
		return EXIT_FAILURE;
	}
	
	cv::Mat src = cv::imread(argv[1], 0);
	if (src.empty()) {
		fprintf(stderr, "imread fail[%s:%d].\n", __FILE__, __LINE__);
		return EXIT_FAILURE;
	}
	
	cv::Mat _src;
	src.convertTo(_src, CV_16UC1);
		
	cv::namedWindow("BEV", 1);
	int rotx = 59;
	int roty = 90;
	int rotz = 90;
	int bevh = 65;
	cv::createTrackbar("rotx", "BEV", &rotx, 180);
	cv::createTrackbar("roty", "BEV", &roty, 180);
	cv::createTrackbar("rotz", "BEV", &rotz, 180);
	cv::createTrackbar("height", "BEV", &bevh, 2000);
	
	Camera camera;
	int keyboard = 0;
	float Pi = 3.141592653f;
	
	while (27 != keyboard && 'q' != keyboard) {
		float _rotx = (rotx - 90) * Pi / 180;
		float _roty = (roty - 90) * Pi / 180;
		float _rotz = (rotz - 90) * Pi / 180;
		
		cv::Mat dst(src.size(), CV_16UC1);
		CameraCreate(&camera, 640, 480, 150, 1.75f, (float)bevh, _rotx, _roty, _rotz);
		CameraPerspectiveTransform(&camera, _src.data, dst.data, src.cols, src.rows);
		
		FILE *fp = fopen("BEV.txt", "w");
		fprintf(fp, "bevh=%d\n", bevh);
		fprintf(fp, "rx=%.5f\n", _rotx);
		fprintf(fp, "ry=%.5f\n", _roty);
		fprintf(fp, "rz=%.5f\n", _rotz);
		fclose(fp);
		
		double minimum, maximum;
		cv::minMaxLoc(dst, &minimum, &maximum);
		cv::Mat norm = 255 * (dst - minimum) / (maximum - minimum);
		cv::Mat u8norm;
		norm.convertTo(u8norm, CV_8UC1);
		
		cv::imshow("BEV", u8norm);
		keyboard = cv::waitKey(10);
	}
	
	return EXIT_SUCCESS;
}

void Help()
{
	printf("Command line usage:\n");
	printf("\t RailwayMonitor <image path>");
}

void test()
{
	cv::Mat image(32, 16, CV_16UC1);
	for (int y = 0; y < 32; ++y) {
		for (int x = 0; x < 16; ++x) {
			*image.ptr<unsigned short>(y, x) = y * 16 + x;
		}
	}

	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	cv::imwrite("test.png", image, compression_params);
}