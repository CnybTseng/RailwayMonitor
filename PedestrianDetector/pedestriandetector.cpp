/** @file pedestriandetector.cpp - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/stat.h>
#include <windows.h>

#include "json/json.h"
#include "opencv2/opencv.hpp"
#include "pthread.h"
#include "fifo.h"
#include "list.h"
#include "HOGEngine.h"
#include "HOGImage.h"
#include "persondetectorwt.tcc"
#include "pedestriandetector.h"

#define RING_IMAGE_NUM					4

struct tagPedestrianDetector
{
	Fifo *rawring;						/*< raw image ring buffer. */
	unsigned short *rawimg;				/*< raw image. */
	float *fltimg;						/*< float type image. */
	Fifo *trackring;					/*< track list ring buffer. */
	char *tracklist;					/*< track list. */
	Camera camera;						/*< camera set. */
	int pdrminx;						/*< left side of pedestrian detection region. */
	int pdrminy;						/*< top side of pedestrian detection region. */
	int pdrmaxx;						/*< right side of pedestrian detection region. */
	int pdrmaxy;						/*< bottom side of pedestrian detection region. */
	float minscale;						/*< minimum scale of image. */
	float maxscale;						/*< maximum scale of image. */
	float svmthresh;					/*< classifier positive threshold. */
	float lmtdist;						/*< look ahead limit distance. */
	Fifo *alarmring;					/*< pedestrian alarm ring buffer. */
	char *alarmlist;					/*< pedestrian alarm list. */
	pthread_t tid;						/*< thread ID of pedestrian detection thread. */
	int disable;						/*< disable pedestrian detector function. */
	int runswitch;						/*< pedestrian detection thread run switch. */
};

/** @name Local functions.
 ** @{ */
static unsigned int RoundupPowerOf2(unsigned int a);
static int InitPedestrianDetectionRegion(const char *path, int *minx, int *miny,
                                         int *maxx, int *maxy);
static int InitImageScaleRange(const char *path, float *minscale, float *maxscale);
static int InitPedestrianAlarmThreshold(const char *path, float *svmth);
static int InitLookaheadLimitDistance(const char *path, float *lmtdist);
static void *PedestrianDetectThread(void *param);
static void ImageUshort2Float(unsigned short *src, int width, int height, float *dst);
static void SetDangerousAlarm(List *alarm, List *track, int yreso, float lmtdist);
static void ViewPedestrianAlarmLine(PedestrianDetector *self);
/** @} */

/** @brief Create a new instance of pedestrian detector.
 ** @param camera camera model.
 ** @return the new instance of pedestrian detector.
 **/
PedestrianDetector *PedestrianDetectorCreate(Camera *camera)
{
	PedestrianDetector *self = (PedestrianDetector *)malloc(sizeof(PedestrianDetector));
	if (!self) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->rawring = NULL;
	self->rawimg = NULL;
	self->fltimg = NULL;
	self->trackring = NULL;
	self->tracklist = NULL;
	self->camera = *camera;
	self->pdrminx = -1;
	self->pdrminy = -1;
	self->pdrmaxx = -1;
	self->pdrmaxy = -1;
	self->minscale = -1;
	self->maxscale = -1;
	self->svmthresh = 0.5f;
	self->lmtdist = self->camera.yreso - 1;
	self->alarmring = NULL;
	self->alarmlist = NULL;
	self->disable = 0;
	self->runswitch = 0;
	
	InitPedestrianDetectionRegion("RailwayMonitor.json", &self->pdrminx, &self->pdrminy,
		&self->pdrmaxx, &self->pdrmaxy);
	InitImageScaleRange("RailwayMonitor.json", &self->minscale, &self->maxscale);
	InitPedestrianAlarmThreshold("RailwayMonitor.json", &self->svmthresh);
	InitLookaheadLimitDistance("RailwayMonitor.json", &self->lmtdist);
	
	// Calculate rounded up image size.
	int ruimgsz = RoundupPowerOf2(self->camera.yreso * self->camera.xreso * sizeof(unsigned short));
	
	self->rawring = fifo_alloc(RING_IMAGE_NUM * ruimgsz);
	if (!self->rawring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->rawimg = (unsigned short *)malloc(ruimgsz);
	if (!self->rawimg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->fltimg = (float *)malloc(camera->xreso * camera->yreso * sizeof(float));
	if (!self->fltimg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->trackring = fifo_alloc(RING_IMAGE_NUM * RoundupPowerOf2(sizeof(List)));
	if (!self->trackring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->tracklist = (char *)malloc(RoundupPowerOf2(sizeof(List)));
	if (!self->tracklist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->alarmring = fifo_alloc(RING_IMAGE_NUM * RoundupPowerOf2(sizeof(List)));
	if (!self->alarmring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->alarmlist = (char *)malloc(RING_IMAGE_NUM * RoundupPowerOf2(sizeof(List)));
	if (!self->alarmlist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		clean:PedestrianDetectorDestroy(self);
	}
	
	return self;
}

/** @brief Put image in ring buffer of pedestrian detector.
 ** @param self pedestrian detector instance.
 ** @param data image data.
 ** @param len bytes of image data.
 ** @return  0 if success,
 **         -1 if fail.
 **/ 
int PedestrianDetectorPutImage(PedestrianDetector *self, unsigned char *data, int len)
{
	int wrt = fifo_put(self->rawring, (const char *)data, len);
	if (wrt != len) {
		return -1;
	}
	
	return 0;
}

/** @brief Put track list in ring buffer of pedestrian detector.
 ** @param self pedestrian detector instance.
 ** @param data track list.
 ** @param len bytes of track list.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int PedestrianDetectorPutTrack(PedestrianDetector *self, char *data, int len)
{
	int wrt = fifo_put(self->trackring, data, len);
	if (wrt != len) {
		return -1;
	}
	
	return 0;
}

/** @brief Begin pedestrian detection thread.
 ** @param self pedestrian detector instance.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int PedestrianDetectorBegin(PedestrianDetector *self)
{
	if (self->runswitch) {
		return 1;
	}

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	int ret = pthread_create(&self->tid, &attr, PedestrianDetectThread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	self->runswitch = 1;
	pthread_attr_destroy(&attr);
	
	return 0;
}

/** @brief Get data from pedestrian detector.
 ** @param self pedestrian detector instance.
 ** @param alarm pedestrian alarm list.
 ** @param len length of alarm list.
 ** @return FIFO read length.
 **/
int PedestrianDetectorGetData(PedestrianDetector *self, char *alarm, int len)
{
	return fifo_get(self->alarmring, alarm, len);
}

/** @brief Switch pedestrian detector on or off.
 ** @param self pedestrian detector instance.
 ** @param onoff on(=1) or off(=0)
 **/
void PedestrianDetectorOnOff(PedestrianDetector *self, int onoff)
{
	self->disable = onoff ^ 0x01;
}

/** @brief Kill pedestrian detection thread.
 ** @param self pedestrian detector isntance.
 **/
void PedestrianDetectorEnd(PedestrianDetector *self)
{
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
	}
}

/** @brief Release all resource of pedestrian detector.
 ** @param self pedestrian detector instance.
 **/
void PedestrianDetectorDestroy(PedestrianDetector *self)
{
	if (self) {
		if (self->rawring) {
			fifo_delete(self->rawring);
		}
		
		if (self->rawimg) {
			free(self->rawimg);
			self->rawimg = NULL;
		}
		
		if (self->fltimg) {
			free(self->fltimg);
			self->fltimg = NULL;
		}
		
		if (self->trackring) {
			fifo_delete(self->trackring);
		}
		
		if (self->tracklist) {
			free(self->tracklist);
			self->tracklist = NULL;
		}
		
		if (self->alarmring) {
			fifo_delete(self->alarmring);
		}
		
		if (self->alarmlist) {
			free(self->alarmlist);
			self->alarmlist = NULL;
		}
		
		free(self);
		self = NULL;
	}
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

/** @brief Initialize pedestrian detection region.
 ** @param path configuration file path.
 ** @param minx left side of detection region.
 ** @param miny top side of detection region.
 ** @param maxx right side of detection region.
 ** @param maxy bottom side of detection region.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitPedestrianDetectionRegion(const char *path, int *minx, int *miny,
                                  int *maxx, int *maxy)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["pedestrianDetectionRegion"].isArray()) {
			*minx = root["pedestrianDetectionRegion"][0].asInt();
			*miny = root["pedestrianDetectionRegion"][1].asInt();
			*maxx = root["pedestrianDetectionRegion"][2].asInt();
			*maxy = root["pedestrianDetectionRegion"][3].asInt();
		} else {
			fprintf(stderr, "pedestrianDetectionRegion isn't array[%s:%d].\n", __FILE__, __LINE__);
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize image scale range.
 ** @param path configuration file path.
 ** @param minscale minimum scale factor.
 ** @param maxscale maximum scale factor.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitImageScaleRange(const char *path, float *minscale, float *maxscale)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["imageScaleRange"].isArray()) {
			*minscale = root["imageScaleRange"][0].asFloat();
			*maxscale = root["imageScaleRange"][1].asFloat();
		} else {
			fprintf(stderr, "imageScaleRange isn't array[%s:%d].\n", __FILE__, __LINE__);
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize pedestrian alarm threshold.
 ** @param path configuration file path.
 ** @param svmth SVM classifier positive threshold.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitPedestrianAlarmThreshold(const char *path, float *svmth)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["SvmThresh"].isDouble()) {
			*svmth = root["SvmThresh"].asFloat();
		} else {
			fprintf(stderr, "SvmThresh isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}


int InitLookaheadLimitDistance(const char *path, float *lmtdist)
{
	std::ifstream ifs(path, std::ios::binary);

	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["maxSumScanLineInter"].isInt()) {
			*lmtdist = root["maxSumScanLineInter"].asInt();
		} else {
			fprintf(stderr, "'maxSumScanLineInter' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Pedestrian detection thread.
 ** @param param thread parameter.
 **/
void *PedestrianDetectThread(void *param)
{
	PedestrianDetector *self = (PedestrianDetector *)param;
	int ruimgsz = RoundupPowerOf2(self->camera.yreso * self->camera.xreso * sizeof(unsigned short));
	int rutklsz = RoundupPowerOf2(sizeof(List));
	int ruarlsz = RoundupPowerOf2(sizeof(List));
	List *tracklist = (List *)self->tracklist;
	List *alarmlist = (List *)self->alarmlist;
	
	HOG::HOGImage *image = new HOG::HOGImage(self->camera.xreso, self->camera.yreso);
	HOG::HOGEngine::Instance()->InitializeHOG(self->camera.xreso, self->camera.yreso,
		PERSON_LINEAR_BIAS, PERSON_WEIGHT_VEC, PERSON_WEIGHT_VEC_LENGTH);
	
	while (self->runswitch) {
		// Read raw image from ring buffer.
		int red = fifo_get(self->rawring, (char *)self->rawimg, ruimgsz);
		if (red != ruimgsz) {
			Sleep(1);
			continue;
		}
		
		ListInit(alarmlist);
		
		if (!self->disable) {		
			// Detect pedestrian in preset regions.
			ImageUshort2Float(self->rawimg, self->camera.xreso, self->camera.yreso, (float *)image->pixels);
			HOG::HOGEngine::Instance()->BeginProcess(image, self->pdrminx, self->pdrminy, self->pdrmaxx,
				self->pdrmaxy, self->minscale, self->maxscale);
			HOG::HOGEngine::Instance()->EndProcess();
			
			for (int j = 0; j < HOG::HOGEngine::Instance()->nmsResultsCount; j++) {
				if (HOG::HOGEngine::Instance()->nmsResults[j].score < self->svmthresh) {
					continue;
				}
				
				struct Alarm *alarm = (struct Alarm *)ListAlloc(sizeof(struct Alarm));
				if (!alarm) exit(-1);
				
				alarm->type = RAILWAY_ALARM_TRACK_PEDESTRIAN;
				alarm->top = HOG::HOGEngine::Instance()->nmsResults[j].y;
				alarm->left = HOG::HOGEngine::Instance()->nmsResults[j].x;
				alarm->bottom = HOG::HOGEngine::Instance()->nmsResults[j].y + HOG::HOGEngine::Instance()->nmsResults[j].height - 1;
				alarm->right = HOG::HOGEngine::Instance()->nmsResults[j].x + HOG::HOGEngine::Instance()->nmsResults[j].width - 1;
				alarm->isdangerous = 0;
				alarm->score = 1 / (1 + exp(self->svmthresh - HOG::HOGEngine::Instance()->nmsResults[j].score));
				alarm->distance = 500;
				alarm->time = time(NULL);
				
				ListAddTail(alarmlist, alarm);
			}
		}
		
		// Read track from ring buffer.
		while (self->runswitch && (red = fifo_get(self->trackring, (char *)tracklist, rutklsz)) != rutklsz) {
			Sleep(1);
		}
		
		if (!self->runswitch) {
			break;
		}
		
		// Set dangerous alarm.
		SetDangerousAlarm(alarmlist, tracklist, self->camera.yreso, self->lmtdist);
			
		// Write pedestrian alarm in ring buffer.
		int wrt = fifo_put(self->alarmring, (const char *)alarmlist, ruarlsz);
		if (wrt != ruarlsz) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		// Update preset regions.
		
		// Debug.
		// ViewPedestrianAlarmLine(self);
		
		ListDelAll(tracklist);
	}
	
	delete image;
	HOG::HOGEngine::Instance()->FinalizeHOG();
	delete HOG::HOGEngine::Instance();
	
	return 0;
}

/** @brief Convert a unsigned short single channel image to float single channel image.
 ** @param src source single channel unsigned short image.
 ** @param width image width.
 ** @param height image height.
 ** @param dst destination single channel float image.
 **/
void ImageUshort2Float(unsigned short *src, int width, int height, float *dst)
{
	cv::Mat _src(height, width, CV_16UC1, src);
	cv::Mat _dst(height, width, CV_32FC1, dst);
	_src.convertTo(_dst, CV_32FC1);
}

/** @brief Set dangerous alarm.
 ** @param alarm pedestrian alarm.
 ** @param track railway track.
 **/
void SetDangerousAlarm(List *alarm, List *track, int yreso, float lmtdist)
{
	struct Node *alarmnode = alarm->head;
	
	int xleftfar = 0;
	int xrightfar = 639;
	struct Node *tracknode = track->head;
	while (tracknode) {
		if (0 == ((struct Track *)tracknode->val)->side) {
			xleftfar = ((struct Track *)tracknode->val)->pwx;
		} else {
			xrightfar = ((struct Track *)tracknode->val)->pwx;
		}
		
		tracknode = tracknode->next;
	}
	
	while (alarmnode) {
		struct Alarm *alarm = (struct Alarm *)alarmnode->val;
		int xfootprint = (alarm->left + alarm->right) >> 1;
		int yfootprint = alarm->bottom;
		int xleftrefer = -1, xrightrefer = -1;
		
		if (yreso - yfootprint > lmtdist) {
			alarmnode = alarmnode->next;
			continue;
		}
		
		struct Node *tracknode = track->head;
		while (tracknode) {
			if ((((struct Track *)tracknode->val)->py == yfootprint) && (0 == ((struct Track *)tracknode->val)->side)) {
				xleftrefer = ((struct Track *)tracknode->val)->pwx;
			}
			
			if ((((struct Track *)tracknode->val)->py == yfootprint) && (1 == ((struct Track *)tracknode->val)->side)) {
				xrightrefer = ((struct Track *)tracknode->val)->pwx;
			}
			
			tracknode = tracknode->next;
		}
		
		if ((-1 != xleftrefer) && (-1 != xrightrefer) &&
			(xfootprint > xleftrefer && xfootprint < xrightrefer)) {
			alarm->isdangerous = 1;
		} else {
			if (xleftfar > 0 && xrightfar < 639 && xfootprint > xleftfar && xfootprint < xrightfar) {
				alarm->isdangerous = 1;
			}
		}
		
		alarmnode = alarmnode->next;
	}
}

/** @brief View pedestrian alarm line.
 ** @param self pedestrian detector instance.
 **/
void ViewPedestrianAlarmLine(PedestrianDetector *self)
{
	cv::Mat _rawimg(self->camera.yreso, self->camera.xreso, CV_16UC1, self->rawimg);

	double minimum, maximum;
	cv::minMaxLoc(_rawimg, &minimum, &maximum);

	cv::Mat norm = 255 * (_rawimg - minimum) / (maximum - minimum);

	cv::Mat gray;
	norm.convertTo(gray, CV_8UC1);

	cv::Mat bgr;
	cv::cvtColor(gray, bgr, CV_GRAY2BGR);

	List *tracklist = (List *)self->tracklist;
	Node *node = tracklist->head;
	while (node) {
		bgr.ptr<cv::Vec3b>(((struct Track *)node->val)->py)[((struct Track *)node->val)->px] = cv::Vec3b(255, 255, 255);
		if (((struct Track *)node->val)->pwx >= 0 && ((struct Track *)node->val)->pwx < self->camera.xreso) {
			bgr.ptr<cv::Vec3b>(((struct Track *)node->val)->py)[((struct Track *)node->val)->pwx] = cv::Vec3b(0, 255, 255);
		}
		node = node->next;
	}
		
	char path[128];
	static int timer = 0;
	sprintf(path, "pedalarmline\\%03d.png", timer++);
	cv::imwrite(path, bgr);
}