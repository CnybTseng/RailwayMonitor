/** @file main.c - Implementation
 ** @brief Test railway track base on extended kalman filter.
 ** @author Zhiwei Zeng
 ** @date 2018.07.24
 **/

/*
Copyright (C) 2018 Zhiwei Zeng.
Copyright (C) 2018 Chengdu ZLT Technology Co., Ltd.
All rights reserved.

This file is part of the cordyceps identification toolkit and is made available under
the terms of the BSD license (see the COPYING file).
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <windows.h>
#include <conio.h>

#include "opencv2/opencv.hpp"
#include "railwaymonitor.h"
#include "pthread.h"
#include "fifo.h"
#include "RDC.h"

typedef enum
{
	FRAME_RESOLUTION_OF_384 = 15,
	FRAME_RESOLUTION_OF_640 = 16,
	PIXEL_FORMAT_YUV_SEMIPLANAR_422 = 22,
	PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 23,
	PIXEL_FORMAT_RGB = 24
}RDC_Sets;

void Help();
unsigned int RoundupPowerOf2(unsigned int a);
int CreateMonitorOutputViewThread();
void *MonitorOutputViewThread(void *s);

int width = 640;
int height = 480;
int over = 0;
Fifo *rawring = NULL;
pthread_t tid;

int main(int argc, char *argv[])
{
	if (argc < 7) {
		Help();
		return EXIT_FAILURE;
	}
	
	char path[128];
	int pnum = 10;
	
	// Parse arguments.
	for (int i = 1; i < argc; i++) {
		if (0 == strcmp(argv[i], "-width")) {
			width = atoi(argv[++i]);
			printf("width=%d.\n", width);
		} else if (0 == strcmp(argv[i], "-height")) {
			height = atoi(argv[++i]);
			printf("height=%d.\n", height);
		} else if (0 == strcmp(argv[i], "-path")) {
			strcpy(path, argv[++i]);
			printf("path=%s.\n", path);
		} else if (0 == strcmp(argv[i], "-pnum")) {
			pnum = atoi(argv[++i]);
			printf("pnum=%d.\n", pnum);
		}
	}
	
	// Timer.
	LARGE_INTEGER freq;
	LARGE_INTEGER start = {0, 0};
	LARGE_INTEGER stop = {0, 0};
	QueryPerformanceFrequency(&freq); 

	FILE *fp = fopen(path, "rb");
	if (!fp) {
		fprintf(stderr, "fopen fail[%s:%d].\n", __FILE__, __LINE__);
		return EXIT_FAILURE;
	}
	
	// Calculate number of frames.
	struct stat statbuf;
	stat(path, &statbuf);
	int nframe = statbuf.st_size / (width * height * sizeof(unsigned short));
	pnum < nframe ? pnum : nframe;

	int len = RoundupPowerOf2(width * height * sizeof(unsigned short));
	unsigned char *rawimg = (unsigned char *)malloc(len);
	if (!rawimg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		return EXIT_FAILURE;
	}
	
	// Create a instance of railway monitor.
	if (RailwayMonitorCreate(width, height, 150)) {
		fprintf(stderr, "RailwayMonitorCreate fail[%s:%d].\n", __FILE__, __LINE__);
		return EXIT_FAILURE;
	}
	
	rawring = fifo_alloc(8 * len);
	if (!rawring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		return EXIT_FAILURE;
	}
		
	RailwayMonitorBegin();
	CreateMonitorOutputViewThread();
	int ch;
	
	for (int t = 0; t < pnum; t++) {
		if (_kbhit()) {
			ch = _getch();
			if (27 == ch) {
				break;
			}
		}
		
		fread(rawimg, sizeof(unsigned short), width * height, fp);
		
		int wrt = fifo_put(rawring, (char *)rawimg, len);
		if (wrt != len) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		// QueryPerformanceCounter(&start);
		if (RailwayMonitorPutData(rawimg, len)) {
			;
		}
				
		// QueryPerformanceCounter(&stop);
		// printf("%lfs.\n", (double)(stop.QuadPart-start.QuadPart)/(double)freq.QuadPart);
		// Sleep(60);
		while (fifo_len(rawring)) {
			Sleep(1);
		}
	}
	
	Sleep(3000);
	RailwayMonitorEnd();
	
	over = 1;
	while (1) {
		int ret = pthread_kill(tid, 0);
		if (ESRCH == ret) {
			break;
		} else if (EINVAL == ret) {
			break;
		} else {
			continue;
		}
	}
	
	if (fp) {
		fclose(fp);
		fp = NULL;
	}
	
	if (rawimg) {
		free(rawimg);
		rawimg = NULL;
	}
	
	if (rawring) {
		fifo_delete(rawring);
	}
	
	RailwayMonitorDestroy();
	
	return EXIT_SUCCESS;
}

void Help()
{
	printf("Command line usage:\n");
	printf("\t RailwayTracker -width <int> -height <int> -path <string> -pnum <int>\n");
}

unsigned int RoundupPowerOf2(unsigned int a)
{
	unsigned int position;
	int i;
	
	if (a == 0) {
		return 0;
	}

	position = 0;
	for (i = a; i != 0; i >>= 1) {
		position++;
	}

	return (unsigned int)(1 << position);
}

int CreateMonitorOutputViewThread()
{
	pthread_attr_t attr;
	int ret;
	
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	ret = pthread_create(&tid, &attr, MonitorOutputViewThread, NULL);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	
	return 0;
}

void *MonitorOutputViewThread(void *s)
{
	RailwayMonitorOutput output;
	int len = RoundupPowerOf2(width * height * sizeof(unsigned short));
	
	char *rawimg = (char *)malloc(len);
	if (!rawimg) {
		return 0;
	}
	
	cv::Mat _rawimg(height, width, CV_16UC1, rawimg);
	cv::Mat norm(height, width, CV_32FC1);
	cv::Mat gray(height, width, CV_8UC1);
	cv::Mat bgr(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
	char path[128];
	int timer = 0;
	unsigned int rol;
	
	RDC_Init(PIXEL_FORMAT_RGB, FRAME_RESOLUTION_OF_640);
	
	int fontface = CV_FONT_HERSHEY_COMPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;
	
#if !defined(_DEBUG)
	time_t tm = time(NULL);
	sprintf(path, "%d.avi", tm);
	cv::VideoWriter vwrt(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, cv::Size(width, height));
#endif
	
	while (!over) {
		int red = fifo_get(rawring, rawimg, len);
		if (red != len) {
			Sleep(1);
			continue;
		}
		
		RDC_SendRawData((unsigned char *)rawimg, width * height * sizeof(unsigned short));
		RDC_GetFrame(bgr.data, &rol);
		
		// Read railway monitor output.
		while (!over && RailwayMonitorGetData(&output)) {
			Sleep(1);
		}
		
		if (over) {
			break;
		}
		
		int counter = 0;
		Node *tracknode = output.track.head;
		while (tracknode) {
			// Track boundary.
			bgr.ptr<cv::Vec3b>(((struct Track *)tracknode->val)->py)[((struct Track *)tracknode->val)->px] = cv::Vec3b(255, 255, 255);
			
			// Obstacle alarm line.
			// if (tracknode->track.owx >= 0 && tracknode->track.owx < width) {
			// 	bgr.ptr<cv::Vec3b>(tracknode->track.py)[tracknode->track.owx] = cv::Vec3b(0, 255, 255);
			// }
			
			// Pedestrian alarm line.
			if (((struct Track *)tracknode->val)->pwx >= 0 && ((struct Track *)tracknode->val)->pwx < width) {
				// bgr.ptr<cv::Vec3b>(tracknode->track.py)[tracknode->track.pwx] = cv::Vec3b(0, 255, 255);
				int pwx = ((struct Track *)tracknode->val)->pwx;
				if (1 == ((struct Track *)tracknode->val)->side) {
					pwx += 5;
					pwx = (pwx < width) ? pwx : width - 1;
				} else {
					pwx -= 5;
					pwx = (pwx >= 0) ? pwx : 0;
				}
				
				bgr.ptr<cv::Vec3b>(((struct Track *)tracknode->val)->py)[pwx] = cv::Vec3b(0, 255, 255);
			}
			
			// if (tracknode->track.side == 0 && ++counter % 24 == 0) {
			// 	char text[128];
			// 	sprintf(text, "%003dm", (int)tracknode->track.dist);
			// 	cv::Point pos(8, tracknode->track.py);
			// 	putText(bgr, text, pos, fontface, scale, cv::Scalar(255, 255, 255), thickness, 8, false);
			// }
			
			tracknode = tracknode->next;
		}
		
		Node *alarmnode = output.alarm.head;
		while (alarmnode) {
			struct Alarm *alarm = (struct Alarm *)alarmnode->val;
			cv::Rect rect(alarm->left, alarm->top, alarm->right - alarm->left, alarm->bottom - alarm->top);
			
			if (alarm->isdangerous) {
				cv::rectangle(bgr, rect, cv::Scalar(0, 0, 255), 1);
			} else {
				// cv::rectangle(bgr, rect, cv::Scalar(0, 0, 255));
				alarmnode = alarmnode->next;
				continue;
			}
			
			// char text[128];
			// sprintf(text, "%.2f", alarm->score);
			// int ytext = alarm->top >= 2 ? alarm->top - 2 : 0;
			// cv::Point pos(alarm->left, ytext);
			// cv::Size sz = cv::getTextSize(text, fontface, scale, thickness, &baseline);
			// cv::rectangle(bgr, pos + cv::Point(0, baseline), pos + cv::Point(sz.width, -sz.height), 
			// 	cv::Scalar(0, 0, 255), CV_FILLED);
			// putText(bgr, text, pos, fontface, scale, cv::Scalar(255, 255, 255), thickness, 8, false);
			
			alarmnode = alarmnode->next;
		}

#ifdef _DEBUG		
		sprintf(path, "merge\\%03d.png", timer++);
		cv::imwrite(path, bgr);
#else
		vwrt << bgr;
#endif
		
		ListDelAll(&output.track);
		ListDelAll(&output.alarm);
	}

	if (rawimg) {
		free(rawimg);
		rawimg = NULL;
	}
	
	return 0;
}