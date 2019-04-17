/** @file railwaymonitor.cpp - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/stat.h>
#include <windows.h>

#include "json/json.h"
#include "pthread.h"
#include "camera.h"
#include "fifo.h"
#include "opencv2/opencv.hpp"
#include "railwaytracker.h"
#include "obstacledetector.h"
#include "pedestriandetector.h"
#include "railwaymonitor.h"

#define RING_IMAGE_NUM					4

struct tagRailwayMonitor
{
	ObstacleDetector *odetector;		/*< obstacle detector. */
	PedestrianDetector *pdetector;		/*< pedestrian detector. */
	RailwayTracker *tracker;			/*< railway tracker. */
	int odetonoff;						/*< switch obstacle detector on or off. */
	int pdetonoff;						/*< switch pedestrian detector on or off. */
	int trkronoff;						/*< switch railway tracker on or off. */
	Camera camera;						/*< camera set. */
	char *tracklist;					/*< track list. */
	char *obsalarmlist;					/*< obstacle alarm list. */
	char *pedalarmlist;					/*< pedestrian alarm list. */
	char *alarmlist;					/*< combined alarm list. */
	char *ruout1;						/*< railway monitor output. */
	char *ruout2;						/*< railway monitor output. */
	Fifo *outring;						/*< railway monitor output ring buffer. */
	pthread_t tid;						/*< thread ID of railway monitor thread. */
	pthread_t tidconf;					/*< thread ID of configuration update thread. */
	int runswitch;						/*< railway monitor thread run switch. */
};

/** @typedef struct RailwayMonitor
 ** @brief railway monitor data structure.
 **/
typedef struct tagRailwayMonitor RailwayMonitor;

/** @typedef PutImgMethod.
 ** @brief the putting image method function pointer.
 **/
typedef int(*PutImgMethod)(void *, unsigned char *, int);

/** @name Local functions.
 ** @{ */
static RailwayMonitor *__RailwayMonitorCreate(Camera *camera);
static int __RailwayMonitorPutData(RailwayMonitor *self, unsigned char *data, int len);
static int __RailwayMonitorBegin(RailwayMonitor *self);
static int __RailwayMonitorGetData(RailwayMonitor *self, RailwayMonitorOutput *output);
static void __RailwayMonitorEnd(RailwayMonitor *self);
static void __RailwayMonitorDestroy(RailwayMonitor *self);
static int InitSubModuleOnOff(const char *path, int *trkronoff, int *pdetonoff, int *odetonoff);
static int InitCameraInstallHeight(const char *path, float *insthei);
static int InitBirdsEyeViewParam(const char *path, float *bevh, float *rx, float *ry, float *rz);
static unsigned int RoundupPowerOf2(unsigned int a);
static int TryToPutImage(PutImgMethod pim, void *receiver, unsigned char *data, int len, int tries);
static void *RailwayMonitorThread(void *param);
static void *ConfigUpdateThread(void *param);
static void ViewTrack(RailwayMonitor *self);
static void PrintSimpleLog(const char *msg);
/** @} */

static RailwayMonitor *monitor = NULL;

/** @brief Create a new instance of railway monitor.
 ** @param xreso horizontal resolution of camera.
 ** @param yreso vertical resolution of camera.
 ** @param lens focal length.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int RailwayMonitorCreate(int xreso, int yreso, int lens)
{
	float insthei = 1.75f;
	InitCameraInstallHeight("RailwayMonitor.json", &insthei);
	
	float bevh = 40;
	float rx = 0, ry = 0, rz = 0;
	InitBirdsEyeViewParam("RailwayMonitor.json", &bevh, &rx, &ry, &rz);
	
	Camera camera;
	CameraCreate(&camera, xreso, yreso, lens, insthei, bevh, rx, ry, rz);
	
	monitor = __RailwayMonitorCreate(&camera);
	if (monitor) {
		return 0;
	}
	
	return -1;
}

/** @brief Put image in ring buffer of railway monitor.
 ** @param data image data.
 ** @param len bytes of image data.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int RailwayMonitorPutData(unsigned char *data, int len)
{
	return __RailwayMonitorPutData(monitor, data,  len);
}

/** @brief Begin railway monitor thread.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int RailwayMonitorBegin()
{
	return __RailwayMonitorBegin(monitor);
}

/** @brief Get data from railway monitor.
 ** @param output railway monitor output.
 ** @return  0 if sucess,
 **         -1 if fail.
 **/
int RailwayMonitorGetData(RailwayMonitorOutput *output)
{
	return __RailwayMonitorGetData(monitor, output);
}

/** @brief Kill railway monitor thread.
 **/
void RailwayMonitorEnd()
{
	__RailwayMonitorEnd(monitor);
}

/** @brief Release all resource of railway monitor.
 **/
void RailwayMonitorDestroy()
{
	__RailwayMonitorDestroy(monitor);
}

/** @brief Create a new instance of railway monitor.
 ** @param camera camera model.
 ** @return the new instance of railway monitor.
 **/
RailwayMonitor *__RailwayMonitorCreate(Camera *camera)
{
	RailwayMonitor *self = (RailwayMonitor *)malloc(sizeof(RailwayMonitor));
	if (!self) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->odetector = NULL;
	self->pdetector = NULL;
	self->tracker = NULL;
	self->odetonoff = 1;
	self->pdetonoff = 1;
	self->trkronoff = 1;
	self->camera = *camera;
	self->tracklist = NULL;
	self->obsalarmlist = NULL;
	self->pedalarmlist = NULL;
	self->alarmlist = NULL;
	self->ruout1 = NULL;
	self->ruout2 = NULL;
	self->outring = NULL;
	self->runswitch = 0;
	
	InitSubModuleOnOff("RailwayMonitor.json", &self->trkronoff, &self->pdetonoff, &self->odetonoff);
	
	self->odetector = ObstacleDetectorCreate(camera);
	if (!self->odetector) {
		goto clean;
	}
	
	self->pdetector = PedestrianDetectorCreate(camera);
	if (!self->pdetector) {
		goto clean;
	}
	
	self->tracker = RailwayTrackerCreate(camera, self->pdetector, self->odetector);
	if (!self->tracker) {
		goto clean;
	}
	
	self->tracklist = (char *)malloc(RoundupPowerOf2(sizeof(List)));
	if (!self->tracklist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->obsalarmlist = (char *)malloc(RoundupPowerOf2(sizeof(List)));
	if (!self->obsalarmlist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->pedalarmlist = (char *)malloc(RoundupPowerOf2(sizeof(List)));
	if (!self->pedalarmlist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->alarmlist = (char *)malloc(RoundupPowerOf2(sizeof(List)));
	if (!self->alarmlist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->ruout1 = (char *)malloc(RoundupPowerOf2(sizeof(RailwayMonitorOutput)));
	if (!self->ruout1) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->ruout2 = (char *)malloc(RoundupPowerOf2(sizeof(RailwayMonitorOutput)));
	if (!self->ruout2) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->outring = fifo_alloc(RING_IMAGE_NUM * RoundupPowerOf2(sizeof(RailwayMonitorOutput)));
	if (!self->outring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		clean:__RailwayMonitorDestroy(self);
	}
	
	return self;
}

/** @brief Put image in ring buffer of railway monitor.
 ** @param self railway monitor instance.
 ** @param data image data.
 ** @param len bytes of image data.
 ** @return  0 if success,
 **         -1 if fail.
 **/ 
int __RailwayMonitorPutData(RailwayMonitor *self, unsigned char *data, int len)
{	
	if (TryToPutImage((PutImgMethod)RailwayTrackerPutData, self->tracker, data, len, 100)) {
		fprintf(stderr, "RailwayTrackerPutData fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	if (TryToPutImage((PutImgMethod)ObstacleDetectorPutImage, self->odetector, data, len, 100)) {
		fprintf(stderr, "ObstacleDetectorPutImage fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	if (TryToPutImage((PutImgMethod)PedestrianDetectorPutImage, self->pdetector, data, len, 100)) {
		fprintf(stderr, "PedestrianDetectorPutImage fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Begin railway monitor thread.
 ** @param self railway monitor instance.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int __RailwayMonitorBegin(RailwayMonitor *self)
{
	if (self->runswitch) {
		return 1;
	}

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	int ret = pthread_create(&self->tid, &attr, RailwayMonitorThread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	ret = pthread_create(&self->tidconf, &attr, ConfigUpdateThread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	self->runswitch = 1;
	pthread_attr_destroy(&attr);
	
	RailwayTrackerOnOff(self->tracker, self->trkronoff);
	ObstacleDetectorOnOff(self->odetector, self->odetonoff);
	PedestrianDetectorOnOff(self->pdetector, self->pdetonoff);
	
	RailwayTrackerBegin(self->tracker);
	ObstacleDetectorBegin(self->odetector);
	PedestrianDetectorBegin(self->pdetector);
	
	return 0;
}

/** @brief Get data from railway monitor.
 ** @param self railway monitor instance.
 ** @param output railway monitor output.
 ** @return  0 if sucess,
 **         -1 if fail.
 **/
int __RailwayMonitorGetData(RailwayMonitor *self, RailwayMonitorOutput *output)
{
	int tries = 100;
	int ruotlsz = RoundupPowerOf2(sizeof(RailwayMonitorOutput));
	while (tries--) {
		if (fifo_get(self->outring, self->ruout2, ruotlsz) == ruotlsz) {
			break;
		}
		Sleep(1);
	}
	
	ListInit(&output->track);
	ListInit(&output->alarm);
	
	if (-1 == tries) {
		return -1;
	}
	
	memcpy(output, self->ruout2, sizeof(RailwayMonitorOutput));
	
	return 0;
}

/** @brief Kill railway monitor thread.
 ** @param self railway monitor isntance.
 **/
void __RailwayMonitorEnd(RailwayMonitor *self)
{
	RailwayTrackerEnd(self->tracker);
	ObstacleDetectorEnd(self->odetector);	
	PedestrianDetectorEnd(self->pdetector);	
	self->runswitch = 0;

	while (1) {
		int ret = pthread_kill(self->tid, 0);
		if (ESRCH == ret) {
			fprintf(stderr, "the thread didn't exists or already quit[%s:%d].\n", __FILE__, __LINE__);
			return;
		} else if (EINVAL == ret) {
			fprintf(stderr, "signal is invalid[%s:%d].\n", __FILE__, __LINE__);
			return;
		} else {
			continue;
		}
		
		ret = pthread_kill(self->tidconf, 0);
		if (ESRCH == ret) {
			fprintf(stderr, "the thread didn't exists or already quit[%s:%d].\n", __FILE__, __LINE__);
			return;
		} else if (EINVAL == ret) {
			fprintf(stderr, "signal is invalid[%s:%d].\n", __FILE__, __LINE__);
			return;
		} else {
			continue;
		}
	}
}

/** @brief Release all resource of railway monitor.
 ** @param self railway monitor instance.
 **/
void __RailwayMonitorDestroy(RailwayMonitor *self)
{
	if (self) {
		RailwayTrackerDestroy(self->tracker);
		
		ObstacleDetectorDestroy(self->odetector);
		
		PedestrianDetectorDestroy(self->pdetector);
		
		if (self->tracklist) {
			free(self->tracklist);
			self->tracklist = NULL;
		}
		
		if (self->obsalarmlist) {
			free(self->obsalarmlist);
			self->obsalarmlist = NULL;
		}
		
		if (self->pedalarmlist) {
			free(self->pedalarmlist);
			self->pedalarmlist = NULL;
		}
		
		if (self->alarmlist) {
			free(self->alarmlist);
			self->alarmlist = NULL;
		}
		
		if (self->ruout1) {
			free(self->ruout1);
			self->ruout1 = NULL;
		}
		
		if (self->ruout2) {
			free(self->ruout2);
			self->ruout2 = NULL;
		}
		
		if (self->outring) {
			fifo_delete(self->outring);
		}
		
		free(self);
		self = NULL;
	}
}

/** @brief Initialize sub-module states.
 ** @param path configuration file path.
 ** @param trkronoff tracker on or off.
 ** @param pdetonoff pedestrian detector on or off.
 ** @param odetonoff obstacle detector on or off.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitSubModuleOnOff(const char *path, int *trkronoff, int *pdetonoff, int *odetonoff)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["trackerOnOff"].isInt()) {
			*trkronoff = root["trackerOnOff"].asInt();
		} else {
			fprintf(stderr, "'trackerOnOff' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["pedestrianDetectorOnOff"].isInt()) {
			*pdetonoff = root["pedestrianDetectorOnOff"].asInt();
		} else {
			fprintf(stderr, "'pedestrianDetectorOnOff' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["obstacleDetectorOnOff"].isInt()) {
			*odetonoff = root["obstacleDetectorOnOff"].asInt();
		} else {
			fprintf(stderr, "'obstacleDetectorOnOff' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize camera installation height.
 ** @param path configuration file path.
 ** @param insthei camera installation height.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitCameraInstallHeight(const char *path, float *insthei)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["cameraInstallHeight"].isDouble()) {
			*insthei = root["cameraInstallHeight"].asFloat();
		} else {
			fprintf(stderr, "'cameraInstallHeight' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize bird's eye view parameters.
 ** @param path configuration file path.
 ** @param bevh bird's eye view height.
 ** @param rx rotation angle around x axis.
 ** @param ry rotation angle around y axis.
 ** @param rz rotation angle around z axis.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitBirdsEyeViewParam(const char *path, float *bevh, float *rx, float *ry, float *rz)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["birdsEyeViewHeight"].isDouble()) {
			*bevh = root["birdsEyeViewHeight"].asFloat();
		} else {
			fprintf(stderr, "'birdsEyeViewHeight' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["rotationX"].isDouble()) {
			*rx = root["rotationX"].asFloat();
		} else {
			fprintf(stderr, "'rotationX' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["rotationY"].isDouble()) {
			*ry = root["rotationY"].asFloat();
		} else {
			fprintf(stderr, "'rotationY' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["rotationZ"].isDouble()) {
			*rz = root["rotationZ"].asFloat();
		} else {
			fprintf(stderr, "'rotationZ' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Round up a number to power of 2.
 ** @param a a integer.
 ** @return a number power of 2.
 **/
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

/** @brief Try to put image in ring buffer.
 ** @param pim putting image method.
 ** @param receiver image data receiver.
 ** @param data image data.
 ** @param len image data length.
 ** @param tries maximum number of tries.
 ** @return  0 if sucess,
 **         -1 if fail.
 **/
int TryToPutImage(PutImgMethod pim, void *receiver, unsigned char *data, int len, int tries)
{
	int counter = (tries > 1 ? tries : 1);
	while (counter--) {
		if (!pim(receiver, data, len)) {
			break;
		}
		Sleep(1);
	}

	if (-1 == counter) {
		return -1;
	}

	return 0;
}

/** @brief Railway monitor thread.
 ** @param param thread parameter.
 **/
void *RailwayMonitorThread(void *param)
{
	RailwayMonitor *self = (RailwayMonitor *)param;
	int rutklsz = RoundupPowerOf2(sizeof(List));
	int ruarlsz = RoundupPowerOf2(sizeof(List));
	int ruotlsz = RoundupPowerOf2(sizeof(RailwayMonitorOutput));
	List *tracklist = (List *)self->tracklist;
	List *obsalarmlist = (List *)self->obsalarmlist;
	List *pedalarmlist = (List *)self->pedalarmlist;
	List *alarmlist = (List *)self->alarmlist;
	RailwayMonitorOutput *output = (RailwayMonitorOutput *)self->ruout1;
	
	while (self->runswitch) {
		// Read track from ring buffer of railway tracker.
		int red = RailwayTrackerGetData(self->tracker, (char *)tracklist, rutklsz);
		if (red != rutklsz) {
			Sleep(1);
			continue;
		}

		// Read obstacle alarm from ring buffer of obstacle detector.
		while (self->runswitch && (red = ObstacleDetectorGetData(self->odetector,
			(char *)obsalarmlist, ruarlsz)) != ruarlsz) {
			Sleep(1);
		}

		if (!self->runswitch) {
			break;
		}

		// Read pedestrian alarm from ring buffer of pedestrian detector.
		while (self->runswitch && (red = PedestrianDetectorGetData(self->pdetector,
			(char *)pedalarmlist, ruarlsz)) != ruarlsz) {
			Sleep(1);
		}

		if (!self->runswitch) {
			break;
		}
		
		// Combine obstacle and pedestrian alarm.
		*alarmlist = *obsalarmlist;
		ListConcatenate(alarmlist, pedalarmlist);
		
		// Package monitor output.
		output->track = *tracklist;
		output->alarm = *alarmlist;
		
		// Write monitor output in ring buffer.
		int wrt = fifo_put(self->outring, (const char *)output, ruotlsz);
		if (wrt != ruotlsz) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
			PrintSimpleLog("Write monitor output in ring buffer fail\n");
		}
		
		// Debug.
		// ViewTrack(self);
		// TrackListDelAll(tracklist);
		// ListDelAll(obsalarmlist);
		// ListDelAll(pedalarmlist);
	}
	
	return 0;
}

/** @brief Configuration update thread.
 ** @param param thread parameter.
 **/
void *ConfigUpdateThread(void *param)
{
	RailwayMonitor *self = (RailwayMonitor *)param;
	const char filename[] = "RailwayMonitor.json";
	time_t last_modify_time = time(NULL);
	struct stat file_stat_buf;
	
	while (self->runswitch) {
		if (!stat(filename, &file_stat_buf)) {
			if (last_modify_time != file_stat_buf.st_mtime) {
				InitSubModuleOnOff(filename, &self->trkronoff, &self->pdetonoff, &self->odetonoff);
				RailwayTrackerOnOff(self->tracker, self->trkronoff);
				ObstacleDetectorOnOff(self->odetector, self->odetonoff);
				PedestrianDetectorOnOff(self->pdetector, self->pdetonoff);
			}
			last_modify_time = file_stat_buf.st_mtime;
		}

		Sleep(1000);
	}
	
	return 0;
}

/** @brief View track.
 ** @param self railway monitor instance.
 **/
void ViewTrack(RailwayMonitor *self)
{
	cv::Mat bgr(self->camera.yreso, self->camera.xreso, CV_8UC3, cv::Scalar(0, 0, 0));

	List *tracklist = (List *)self->tracklist;
	Node *node = tracklist->head;
	while (node) {
		bgr.ptr<cv::Vec3b>(((struct Track *)node->val)->py)[((struct Track *)node->val)->px] = cv::Vec3b(255, 255, 255);
		node = node->next;
	}
	
	List *obsalarmlist = (List *)self->obsalarmlist;
	Node *alarmnode = obsalarmlist->head;
	while (alarmnode) {
		struct Alarm *alarm = (struct Alarm *)alarmnode->val;
		cv::Rect rect(alarm->left, alarm->top, alarm->right - alarm->left, alarm->bottom - alarm->top);
		cv::rectangle(bgr, rect, cv::Scalar(0, 0, 255));
		alarmnode = alarmnode->next;
	}
		
	char path[128];
	static int timer = 0;
	sprintf(path, "merge\\%03d.png", timer++);
	cv::imwrite(path, bgr);
}

void PrintSimpleLog(const char *msg)
{
	FILE *fp = fopen("railwaymonitor.txt", "a");
	fprintf(fp, msg);
	fclose(fp);
}