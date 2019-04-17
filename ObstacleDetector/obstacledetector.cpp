/** @file obstacledetector.cpp - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/stat.h>
#include <windows.h>

#include "json/json.h"
#include "pthread.h"
#include "fifo.h"
#include "list.h"
#include "opencv2/opencv.hpp"
#include "obstacledetector.h"

#define RING_IMAGE_NUM					4

#define ODMin(a, b) (((a) < (b)) ? (a) : (b))
#define ODMax(a, b) (((a) > (b)) ? (a) : (b))

/** @typedef struct ObstacleDetectRegion
 ** @brief obstacle detection region data structure.
 **/
typedef struct
{
	int leftx;							/*< x of left side. */
	int topy;							/*< y of top side. */
	int width;							/*< region width. */
	int height;							/*< region height. */
	union {
		float dbright;					/*< differences in brightness between neighboring windows. */
		struct {
			float var;					/*< variance between neighboring windows. */
			float corr;					/*< correlations between neighboring windows. */
		} tex;
	};
} ObstacleDetectRegion;

struct tagObstacleDetector
{
	Fifo *rawring;						/*< raw image ring buffer. */
	unsigned short *rawimg;				/*< raw image. */
	unsigned short *bevimg;				/*< bird's eye view image. */
	unsigned short *hshimg;				/*< horizontal offset image. */
	float *hshimgf32;					/*< float type horizontal offset image. */
	Fifo *trackring;					/*< track list ring buffer. */
	char *tracklist;					/*< track list. */
	Camera camera;						/*< camera set. */
	float yaw;							/*< yaw angle. */
	float pitch;						/*< pitch angle. */
	float latoffset;					/*< lateral offset. */
	float lanewidth;					/*< lane width. */
	List bevtracklist;				/*< perspective transformed track list. */
	int *bevobswarnx;					/*< perspective transformed x-outer side of obstacle warning region. */
	int *cx;							/*< x position of center line of track. */
	int *offset;						/*< horizontal offset table. */
	int *offsetstate;					/*< offset set(=1) or not(=0). */
	int wcdr;							/*< width of continuity detection region. */
	int hcdr;							/*< height of continuity detection region. */
	ObstacleDetectRegion *cdreg;		/*< continuity detection region. */
	int nlcdr;							/*< number of left continuity detection region. */
	int nrcdr;							/*< number of right continuity detection region. */
	int wtdr;							/*< width of texture detection region. */
	int htdr;							/*< height of texture detection region. */
	float dbrth;						/*< difference threshold in brightness between neighboring regions. */
	ObstacleDetectRegion *tdreg;		/*< texture detection region. */
	int ntdr;							/*< number of texture detection region. */
	float varth;						/*< difference threshold of the variance of two consecutive regions. */
	float corrth;						/*< correlation threshold of the two consecutive regions. */
	Fifo *alarmring;					/*< obstacle alarm ring buffer. */
	char *alarmlist;					/*< obstacle alarm list. */
	pthread_t tid;						/*< thread ID of obstacle detection thread. */
	int disable;						/*< disable obstacle detector function. */
	int runswitch;						/*< obstacle detection thread run switch. */
};

/** @name Local functions.
 ** @{ */
static unsigned int RoundupPowerOf2(unsigned int a);
static int InitCameraInstallParam(const char *path, float *yaw, float *pitch,
                                  float *latoffset, float *lanewidth);
static int InitContinuityDetectRegionSize(const char *path, int *wcdr, int *hcdr);
static int InitTextureDetectRegionSize(const char *path, int *wtdr, int *htdr);
static int InitObstacleAlarmThreshold(const char *path, float *dbrth, float *varth, float *corrth);
static void *ObstacleDetectThread(void *param);
static void CalBEVTrack(Camera *camera, List *src, List *dst);
static void CalBEVObstacleWarnLine(Camera *camera, List *track, int *bevobswarnx);
static void SetObstacleWarnLineOnTrack(List *track, int rb, int *bevobswarnx);
static void FillTrackGap(List *track, struct Track *a, struct Track *b);
static void CalHorizontalOffsetOfTrack(List *bevtracklist, int *cx,
                                       int *offset, int *offsetstate, int len);
static void AlignBEVTrack(List *bevtrack, int *offsetstate);
static void HorizonShiftImage(unsigned short *bevimg, unsigned short *hshimg,
                              int *offset, int width, int height);
static void HorizonShiftTrack(List *bevtracklist, int *offset);
static void SetContinuityDetectRegion(List *bevtrack, int wcdr, int hcdr,
                                      ObstacleDetectRegion *cdreg, int *nlcdr, int *nrcdr);
static void CalDiffOfBright(unsigned short *hshimg, int width, int height,
                            ObstacleDetectRegion *cdreg, int nlcdr, int nrcdr);
static void SetTextureDetectRegion(List *bevtrack, int wtdr, int htdr,
                                   ObstacleDetectRegion *tdreg, int *ntdr);
static void CalDiffOfVariance(unsigned short *hshimg, int width, int height,
                              ObstacleDetectRegion *tdreg, int ntdr);
static void ImageUshort2Float(unsigned short *src, int width, int height, float *dst);
static void CalCorrelation(float *hshimg, int width, int height,
                           ObstacleDetectRegion *tdreg, int ntdr);
static void ContinuityDiscrimination(ObstacleDetectRegion *cdreg, int nlcdr, int nrcdr,
                                     Camera *camera, int *offset, float dbrth, List *alarmlist);
static void TextureDiscrimination(ObstacleDetectRegion *tdreg, int ntdr, Camera *camera,
                                  int *offset, float varth, float corrth, List *alarmlist);
static void ViewObstacleAlarmLine(ObstacleDetector *self);
static void ViewBEVImage(ObstacleDetector *self);
/** @} */

/** @brief Create a new instance of obstacle detector.
 ** @param camera camera model.
 ** @return the new instance of obstacle detector.
 **/
ObstacleDetector *ObstacleDetectorCreate(Camera *camera)
{
	ObstacleDetector *self = (ObstacleDetector *)malloc(sizeof(ObstacleDetector));
	if (!self) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->rawring = NULL;
	self->rawimg = NULL;
	self->bevimg = NULL;
	self->hshimg = NULL;
	self->hshimgf32 = NULL;
	self->trackring = NULL;
	self->tracklist = NULL;
	self->camera = *camera;
	self->yaw = 0;
	self->pitch = 0;
	self->latoffset = 0;
	self->lanewidth = 1.435f;
	self->bevobswarnx = 0;
	self->cx = NULL;
	self->offset = NULL;
	self->offsetstate = NULL;
	self->wcdr = 0;
	self->hcdr = 16;
	self->cdreg = NULL;
	self->nlcdr = 0;
	self->nrcdr = 0;
	self->wtdr = 0;
	self->htdr = 16;
	self->dbrth = 2.6f;
	self->tdreg = NULL;
	self->ntdr = 0;
	self->varth = 6400;
	self->corrth = 0.5f;
	self->alarmring = NULL;
	self->alarmlist = NULL;
	self->disable = 0;
	self->runswitch = 0;
	
	if (InitCameraInstallParam("RailwayMonitor.json", &self->yaw, &self->pitch, &self->latoffset,
		&self->lanewidth)) {
		goto clean;
	}

	CalRotationAngleOfCam(&self->camera, self->yaw, self->pitch, self->latoffset, self->lanewidth);
	// CalPerTransfMatOfCam(&self->camera, self->yaw, self->pitch, self->latoffset, self->lanewidth);
	
	InitContinuityDetectRegionSize("RailwayMonitor.json", &self->wcdr, &self->hcdr);
	InitTextureDetectRegionSize("RailwayMonitor.json", &self->wtdr, &self->htdr);
	InitObstacleAlarmThreshold("RailwayMonitor.json", &self->dbrth, &self->varth, &self->corrth);
	
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
	
	self->bevimg = (unsigned short *)malloc(self->camera.yreso * self->camera.xreso * sizeof(unsigned short));
	if (!self->bevimg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->hshimg = (unsigned short *)malloc(self->camera.yreso * self->camera.xreso * sizeof(unsigned short));
	if (!self->hshimg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->hshimgf32 = (float *)malloc(self->camera.yreso * self->camera.xreso * sizeof(float));
	if (!self->hshimgf32) {
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
	
	self->bevobswarnx = (int *)malloc((camera->yreso << 1) * sizeof(int));
	if (!self->bevobswarnx) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->cx = (int *)malloc(camera->yreso * sizeof(int));
	if (!self->cx) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->offset = (int *)malloc(camera->yreso * sizeof(int));
	if (!self->offset) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->offsetstate = (int *)malloc(camera->yreso * sizeof(int));
	if (!self->offsetstate) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->cdreg = (ObstacleDetectRegion *)malloc((camera->yreso << 1) * sizeof(ObstacleDetectRegion));
	if (!self->cdreg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->tdreg = (ObstacleDetectRegion *)malloc(camera->yreso * sizeof(ObstacleDetectRegion));
	if (!self->tdreg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->alarmring = fifo_alloc(RING_IMAGE_NUM * RoundupPowerOf2(sizeof(List)));
	if (!self->alarmring) {
		fprintf(stderr, "fifo_alloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->alarmlist = (char *)malloc(RoundupPowerOf2(sizeof(List)));
	if (!self->alarmlist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		clean:ObstacleDetectorDestroy(self);
	}
	
	return self;
}

/** @brief Put image in ring buffer of obstacle detector.
 ** @param self obstacle detector instance.
 ** @param data image data.
 ** @param len bytes of image data.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int ObstacleDetectorPutImage(ObstacleDetector *self, unsigned char *data, int len)
{
	int wrt = fifo_put(self->rawring, (const char *)data, len);
	if (wrt != len) {
		return -1;
	}
	
	return 0;
}

/** @brief Put track list in ring buffer of obstacle detector.
 ** @param self obstacle detector instance.
 ** @param data track list.
 ** @param len bytes of track list.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int ObstacleDetectorPutTrack(ObstacleDetector *self, char *data, int len)
{
	int wrt = fifo_put(self->trackring, data, len);
	if (wrt != len) {
		return -1;
	}
	
	return 0;
}

/** @brief Begin obstacle detection thread.
 ** @param self obstacle detector instance.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int ObstacleDetectorBegin(ObstacleDetector *self)
{
	if (self->runswitch) {
		return 1;
	}

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	int ret = pthread_create(&self->tid, &attr, ObstacleDetectThread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	self->runswitch = 1;
	pthread_attr_destroy(&attr);
	
	return 0;
}

/** @brief Get data from obstacle detector.
 ** @param self obstacle detector instance.
 ** @param alarm obstacle alarm list.
 ** @param len length of alarm list.
 ** @return FIFO read length.
 **/
int ObstacleDetectorGetData(ObstacleDetector *self, char *alarm, int len)
{
	return fifo_get(self->alarmring, alarm, len);
}

/** @brief Switch obstacle detector on or off.
 ** @param self obstacle detector instance.
 ** @param onoff on(=1) or off(=0)
 **/
void ObstacleDetectorOnOff(ObstacleDetector *self, int onoff)
{
	self->disable = onoff ^ 0x01;
}

/** @brief Kill obstacle detection thread.
 ** @param self obstacle detector isntance.
 **/
void ObstacleDetectorEnd(ObstacleDetector *self)
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

/** @brief Release all resource of obstacle detector.
 ** @param self obstacle detector instance.
 **/
void ObstacleDetectorDestroy(ObstacleDetector *self)
{
	if (self) {
		if (self->rawring) {
			fifo_delete(self->rawring);
		}
		
		if (self->rawimg) {
			free(self->rawimg);
			self->rawimg = NULL;
		}
		
		if (self->bevimg) {
			free(self->bevimg);
			self->bevimg = NULL;
		}
		
		if (self->hshimg) {
			free(self->hshimg);
			self->hshimg = NULL;
		}
		
		if (self->hshimgf32) {
			free(self->hshimgf32);
			self->hshimgf32 = NULL;
		}
		
		if (self->trackring) {
			fifo_delete(self->trackring);
		}
		
		if (self->tracklist) {
			free(self->tracklist);
			self->tracklist = NULL;
		}
		
		if (self->bevobswarnx) {
			free(self->bevobswarnx);
			self->bevobswarnx = NULL;
		}
		
		if (self->cx) {
			free(self->cx);
			self->cx = NULL;
		}
		
		if (self->offset) {
			free(self->offset);
			self->offset = NULL;
		}
		
		if (self->offsetstate) {
			free(self->offsetstate);
			self->offsetstate = NULL;
		}
		
		if (self->cdreg) {
			free(self->cdreg);
			self->cdreg = NULL;
		}
		
		if (self->tdreg) {
			free(self->tdreg);
			self->tdreg = NULL;
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

/** @brief Initialize camera installation parameters.
 ** @param path configuration file path.
 ** @param yaw yaw angle.
 ** @param pitch pitch angle.
 ** @param latoffset lateral offset.
 ** @param lanewidth lane width.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitCameraInstallParam(const char *path, float *yaw, float *pitch,
                           float *latoffset, float *lanewidth)
{
	std::ifstream ifs(path, std::ios::binary);

	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["laneInitState"]["yawAngle"].isDouble()) {
			*yaw = root["laneInitState"]["yawAngle"].asFloat();
		} else {
			fprintf(stderr, "yawAngle isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["pitchAngle"].isDouble()) {
			*pitch = root["laneInitState"]["pitchAngle"].asFloat();
		} else {
			fprintf(stderr, "pitchAngle isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["lateralOffset"].isDouble()) {
			*latoffset = root["laneInitState"]["lateralOffset"].asFloat();
		} else {
			fprintf(stderr, "lateralOffset isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["laneWidth"].isDouble()) {
			*lanewidth = root["laneInitState"]["laneWidth"].asFloat();
		} else {
			fprintf(stderr, "laneWidth isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize continuity detection region size.
 ** @param path configuration file path.
 ** @param wcdr width of continuity detection region.
 ** @param hcdr height of continuity detection region.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitContinuityDetectRegionSize(const char *path, int *wcdr, int *hcdr)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["widthOfContinuityDetectRegion"].isInt()) {
			*wcdr = root["widthOfContinuityDetectRegion"].asInt();
		} else {
			fprintf(stderr, "widthOfContinuityDetectRegion isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["heightOfContinuityDetectRegion"].isInt()) {
			*hcdr = root["heightOfContinuityDetectRegion"].asInt();
		} else {
			fprintf(stderr, "heightOfContinuityDetectRegion isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize texture detection region size.
 ** @param path configuration file path.
 ** @param wtdr width of texture detection region size.
 ** @param htdr heighth of texture detection region size.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitTextureDetectRegionSize(const char *path, int *wtdr, int *htdr)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["widthOfTextureDetectRegion"].isInt()) {
			*wtdr = root["widthOfTextureDetectRegion"].asInt();
		} else {
			fprintf(stderr, "widthOfTextureDetectRegion isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["heightOfTextureDetectRegion"].isInt()) {
			*htdr = root["heightOfTextureDetectRegion"].asInt();
		} else {
			fprintf(stderr, "heightOfTextureDetectRegion isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize obstacle alarm threshold.
 ** @param path configuration file path.
 ** @param dbrth difference threshold in brightness between neighboring regions.
 ** @param varth difference threshold of the variance of two consecutive regions.
 ** @param corrth correlation threshold of the two consecutive regions.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitObstacleAlarmThreshold(const char *path, float *dbrth, float *varth, float *corrth)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["diffBrightThresh"].isDouble()) {
			*dbrth = root["diffBrightThresh"].asFloat();
		} else {
			fprintf(stderr, "diffBrightThresh isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["diffVarThresh"].isDouble()) {
			*varth = root["diffVarThresh"].asFloat();
		} else {
			fprintf(stderr, "diffVarThresh isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["corrThresh"].isDouble()) {
			*corrth = root["corrThresh"].asFloat();
		} else {
			fprintf(stderr, "corrThresh isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Obstacle detection thread.
 ** @param param thread parameter.
 **/
void *ObstacleDetectThread(void *param)
{
	ObstacleDetector *self = (ObstacleDetector *)param;
	int ruimgsz = RoundupPowerOf2(self->camera.yreso * self->camera.xreso * sizeof(unsigned short));
	int rutklsz = RoundupPowerOf2(sizeof(List));
	int ruarlsz = RoundupPowerOf2(sizeof(List));
	List *tracklist = (List *)self->tracklist;
	List *alarmlist = (List *)self->alarmlist;
	
	while (self->runswitch) {
		// Read raw image from ring buffer.
		int red = fifo_get(self->rawring, (char *)self->rawimg, ruimgsz);
		if (red != ruimgsz) {
			Sleep(1);
			continue;
		}
		
		if (!self->disable) {
			// Perspective transform.
			CameraPerspectiveTransform(&self->camera, (unsigned char *)self->rawimg, (unsigned char *)self->bevimg,
				self->camera.xreso, self->camera.yreso);
		}
		
		// Read track from ring buffer.
		while (self->runswitch && (red = fifo_get(self->trackring, (char *)tracklist, rutklsz)) != rutklsz) {
			Sleep(1);
		}
		
		if (!self->runswitch) {
			break;
		}
		
		ListInit(&self->bevtracklist);
		ListInit(alarmlist);
		
		if (self->disable) {
			goto eod;
		}
		
		// Detect track gap.
		
		// Calculate bird's eye view of track.
		CalBEVTrack(&self->camera, tracklist, &self->bevtracklist);
		
		// Calculate bird's eye view of obstacle warning line.
		CalBEVObstacleWarnLine(&self->camera, tracklist, self->bevobswarnx);
		SetObstacleWarnLineOnTrack(&self->bevtracklist, self->camera.yreso, self->bevobswarnx);
		
		// Calculate horizontal offset.
		CalHorizontalOffsetOfTrack(&self->bevtracklist, self->cx, self->offset, self->offsetstate, self->camera.yreso);
		AlignBEVTrack(&self->bevtracklist, self->offsetstate);
		
		// Normalize the bird's eye view of image.
		HorizonShiftImage(self->bevimg, self->hshimg, self->offset, self->camera.xreso, self->camera.yreso);
		
		// Normalize the bird's eye view of track.
		HorizonShiftTrack(&self->bevtracklist, self->offset);
		
		// Set railway continuity detection windows.
		SetContinuityDetectRegion(&self->bevtracklist, self->wcdr, self->hcdr, self->cdreg, &self->nlcdr, &self->nrcdr);
		
		// Calculate differences in brightness between neighboring windows.
		CalDiffOfBright(self->hshimg, self->camera.xreso, self->camera.yreso, self->cdreg, self->nlcdr, self->nrcdr);		
		
		// Track continuity exception discrimination and location.
		ContinuityDiscrimination(self->cdreg, self->nlcdr, self->nrcdr, &self->camera, self->offset, self->dbrth, alarmlist);
		
		// Set track texture detection region.
		SetTextureDetectRegion(&self->bevtracklist, self->wtdr, self->htdr, self->tdreg, &self->ntdr);
		
		// Calculate variance between neighboring windows. 
		CalDiffOfVariance(self->hshimg, self->camera.xreso, self->camera.yreso, self->tdreg, self->ntdr);
		
		// Calculate correlation between neighboring windows. 
		ImageUshort2Float(self->hshimg, self->camera.xreso, self->camera.yreso, self->hshimgf32);
		CalCorrelation(self->hshimgf32, self->camera.xreso, self->camera.yreso, self->tdreg, self->ntdr);
		
		// Texture exception discrimination and location.
		TextureDiscrimination(self->tdreg, self->ntdr, &self->camera, self->offset, self->varth, self->corrth, alarmlist);
		
		eod:
		// Write obstacle alarm information in ring buffer.
		int wrt = fifo_put(self->alarmring, (const char *)alarmlist, ruarlsz);
		if (wrt != ruarlsz) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
#ifdef _DEBUG
		// ViewObstacleAlarmLine(self);
		ViewBEVImage(self);
#endif		
		ListDelAll(tracklist);
		ListDelAll(&self->bevtracklist);
	}
	
	return 0;
}

/** @brief Calculate bird's eye view of track.
 ** @param camera camera model.
 ** @param src source track list.
 ** @param dst destination track list.
 **/
void CalBEVTrack(Camera *camera, List *src, List *dst)
{
	Node *head = src->head;
	struct Track lastrack = {-1, -1, -1, -1, -1, -1};
	while (head) {
		struct Track *track = (struct Track *)ListAlloc(sizeof(struct Track));
		if (!track) exit(-1);
		CameraPerspectiveTransformPerPoint(camera, ((struct Track *)head->val)->px, ((struct Track *)head->val)->py, &track->px, &track->py, 1);
		if (track->px < 0 || track->px > camera->xreso - 1 ||
			track->py < 0 || track->py > camera->yreso - 1) {
			head = head->next;
			continue;
		}
		
		track->mx = -1;
		track->pwx = -1;
		track->owx = -1;
		track->side = ((struct Track *)head->val)->side;
		track->dist = ((struct Track *)head->val)->dist;
		
		// Fill track gap.
		if ((lastrack.side == track->side) && (abs(track->px - lastrack.px) > 1 ||
			abs(track->py - lastrack.py) > 1)) {
			FillTrackGap(dst, &lastrack, track);
		}
		
		ListAddTail(dst, track);
		head = head->next;
		lastrack = *track;
	}
}

/** @brief Calculate bird's eye view of obstacle warning line.
 ** @param camera camera model.
 ** @param track track list.
 ** @param bevobswarnx bird's eye view of obstacle warning line.
 **/
void CalBEVObstacleWarnLine(Camera *camera, List *track, int *bevobswarnx)
{
	memset(bevobswarnx, -1, (camera->yreso << 1) * sizeof(int));
	
	Node *head = track->head;
	while (head) {
		int wx = -1;
		int wy = -1;
		CameraPerspectiveTransformPerPoint(camera, ((struct Track *)head->val)->owx, ((struct Track *)head->val)->py, &wx, &wy, 1);
		if (wy < 0 || wy > camera->yreso - 1) {
			head = head->next;
			continue;
		}
		
		if (0 == ((struct Track *)head->val)->side) {
			bevobswarnx[wy] = wx;
		} else {
			bevobswarnx[wy + camera->yreso] = wx;
		}
		
		head = head->next;
	}
	
	// Fill left obstacle warning line gap.
	for (int i = 0; i < camera->yreso - 1; i++) {
		if (-1 == bevobswarnx[i]) {
			continue;
		}
		
		for (int j = i + 1; j < camera->yreso; j++) {
			if (-1 == bevobswarnx[j]) {
				bevobswarnx[j] = bevobswarnx[i];
			} else {
				break;
			}
		}
	}
	
	// Fill right obstacle warning line gap.
	int total = camera->yreso << 1;
	for (int i = camera->yreso; i < total; i++) {
		if (-1 == bevobswarnx[i]) {
			continue;
		}
		
		for (int j = i + 1; j < total; j++) {
			if (-1 == bevobswarnx[j]) {
				bevobswarnx[j] = bevobswarnx[i];
			} else {
				break;
			}
		}
	}
}

/** @brief Set obstacle warning line on bird's eye view of track.
 ** @param track bird's eye view of track.
 ** @param rb right warning line point index bias.
 ** @param bevobswarnx bird's eye view of obstacle warning line.
 **/
void SetObstacleWarnLineOnTrack(List *track, int rb, int *bevobswarnx)
{
	Node *head = track->head;
	while (head) {
		if (((struct Track *)head->val)->py < 0 || ((struct Track *)head->val)->py > rb - 1) {
			head = head->next;
			continue;
		}
		
		if (0 == ((struct Track *)head->val)->side) {
			((struct Track *)head->val)->owx = bevobswarnx[((struct Track *)head->val)->py];
		} else {
			((struct Track *)head->val)->owx = bevobswarnx[((struct Track *)head->val)->py + rb];
		}
		
		head = head->next;
	}
}

/** @brief Fill track gap.
 ** @param track track list.
 ** @param a track border.
 ** @param b next track border.
 **/
void FillTrackGap(List *track, struct Track *a, struct Track *b)
{
	int dx = abs(b->px - a->px);
	int dy = abs(b->py - a->py);
	
	if (dx > dy) {
		int step = (b->px > a->px ? 1 : -1);
		for (int x = a->px; x != b->px; x += step) {
			int y = (int)((b->py - a->py) * (x - a->px) / ((float)b->px - a->px) + a->py);
			struct Track *m = (struct Track *)ListAlloc(sizeof(struct Track));
			if (!m) exit(-1);
			m->px = x;
			m->py = y;
			m->mx = -1;
			m->pwx = -1;
			m->owx = -1;
			m->side = a->side;
			ListAddTail(track, m);
		}
	} else {
		if (0 == dx) {
			for (int y = a->py; y > b->py; y--) {
				struct Track *m = (struct Track *)ListAlloc(sizeof(struct Track));
				if (!m) exit(-1);
				m->px = a->px;
				m->py = y;
				m->mx = -1;
				m->pwx = -1;
				m->owx = -1;
				m->side = a->side;
				ListAddTail(track, m);
			}
		} else {
			for (int y = a->py; y > b->py; y--) {
				int x = (int)((b->px - a->px) * (y - a->py) / ((float)b->py - a->py) + a->px);
				struct Track *m = (struct Track *)ListAlloc(sizeof(struct Track));
				if (!m) exit(-1);
				m->px = x;
				m->py = y;
				m->mx = -1;
				m->pwx = -1;
				m->owx = -1;
				m->side = a->side;
				ListAddTail(track, m);
			}
		}
	}
}

/** @brief Calculate horizontal offset of track.
 ** @param bevtracklist bird's eye view of track.
 ** @param cx x position of track center line.
 ** @param offset horizontal offset table of track.
 ** @param offsetstate offset table state.
 ** @param len length of offset table.
 **/
void CalHorizontalOffsetOfTrack(List *bevtracklist, int *cx,
                                int *offset, int *offsetstate, int len) 
{
	// Initialize horizontal offset table.
	for (int y = 0; y < len; y++) {
		offset[y] = 0;
		offsetstate[y] = 0;
	}
	
	// Find the start position of right track.
	Node *rightrack = bevtracklist->head;
	while (rightrack && 0 == ((struct Track *)rightrack->val)->side) {
		rightrack = rightrack->next;
	}
		
	if (!rightrack) {
		fprintf(stderr, "right track list is empty[%s:%d].\n", __FILE__, __LINE__);
		return;
	}
	
	int aligned = 0;
	Node *leftrack = bevtracklist->head;
		
	// Align left and right first track node.
	if (((struct Track *)leftrack->val)->py == ((struct Track *)rightrack->val)->py) {
		aligned = 1;
	} else {
		if (((struct Track *)leftrack->val)->py < ((struct Track *)rightrack->val)->py) {
			while (rightrack) {
				if (((struct Track *)leftrack->val)->py == ((struct Track *)rightrack->val)->py) {
					aligned = 1;
					break;
				}
				rightrack = rightrack->next;
			}
		} else {
			while (leftrack) {
				if (((struct Track *)leftrack->val)->py == ((struct Track *)rightrack->val)->py) {
					aligned = 1;
					break;
				}
				leftrack = leftrack->next;
			}
		}
	}
	
	if (!aligned) {
		fprintf(stderr, "left and right track aren't aligned[%s:%d].\n", __FILE__, __LINE__);
		return;
	}
	
	int sumcx = 0;
	int counter = 0;
	int height = len;
	Node *burightrack = rightrack;
	
	// Calculate x position of center line.
	while (leftrack && 0 == ((struct Track *)leftrack->val)->side && rightrack) {
		if (((struct Track *)leftrack->val)->py >= 0 && ((struct Track *)leftrack->val)->py < height) {
			int cxi = (int)((((struct Track *)leftrack->val)->px + ((struct Track *)rightrack->val)->px) / 2.0f + 0.5f);
			cx[((struct Track *)leftrack->val)->py] = cxi;
			sumcx += cxi;
			counter++;
		}
		
		// Skip to the next scan line.
		int y = ((struct Track *)leftrack->val)->py;
		while (leftrack && 0 == ((struct Track *)leftrack->val)->side && ((struct Track *)leftrack->val)->py == y) {
			leftrack = leftrack->next;
		}
		
		while (rightrack && ((struct Track *)rightrack->val)->py == y) {
			rightrack = rightrack->next;	
		}
	}

	int rx = (int)(sumcx / (float)counter + 1e-8f);
	leftrack = bevtracklist->head;
	rightrack = burightrack;
	
	// Calculate horizontal offset of track.
	while (leftrack && 0 == ((struct Track *)leftrack->val)->side && rightrack) {
		if (((struct Track *)leftrack->val)->py >= 0 && ((struct Track *)leftrack->val)->py < height) {
			offset[((struct Track *)leftrack->val)->py] = cx[((struct Track *)leftrack->val)->py] - rx;
			offsetstate[((struct Track *)leftrack->val)->py] = 1;
		}
		
		// Skip to the next scan line.
		int y = ((struct Track *)leftrack->val)->py;
		while (leftrack && 0 == ((struct Track *)leftrack->val)->side && ((struct Track *)leftrack->val)->py == y) {
			leftrack = leftrack->next;
		}
		
		while (rightrack && ((struct Track *)rightrack->val)->py == y) {
			rightrack = rightrack->next;	
		}
	}
}

/** @brief Align bird's eye view of left and right track.
 ** @param bevtrack bird's eye view of track.
 ** @param offsetstate horizontal offset state.
 **/ 
void AlignBEVTrack(List *bevtrack, int *offsetstate)
{
	struct Node *node = bevtrack->head;
	while (node) {
		if (0 == offsetstate[((struct Track *)node->val)->py]) {
			node = ListDelNode(bevtrack, node);
			continue;
		}
		node = node->next;
	}
}

/** @brief Horizontal offset of image.
 ** @param bevimg bird's eye view of image.
 ** @param hshimg horizontal shifted image.
 ** @param offset horizontal offset.
 ** @param width image width.
 ** @param height image height.
 **/
void HorizonShiftImage(unsigned short *bevimg, unsigned short *hshimg,
                       int *offset, int width, int height)
{
	unsigned short *ptr = hshimg;
	
	for (int y = 0; y < height; y++) {
		int os = offset[y];
		for (int x = 0; x < width; x++) {
			int dstx = x + os;
			if (dstx >= 0 && dstx < width) {
				*ptr = *(bevimg + y * width + dstx);
			} else if (dstx < 0){
				*ptr = *(bevimg + y * width);
			} else {
				*ptr = *(bevimg + y * width + width - 1);
			}

			ptr++;
		}
	}
}

/** @brief Horizontal shift track.
 ** @param bevtracklist bird's eye view of track.
 ** @param offset horizontal offset.
 **/
void HorizonShiftTrack(List *bevtracklist, int *offset)
{
	Node *node = bevtracklist->head;
	while (node) {
		((struct Track *)node->val)->px -= offset[((struct Track *)node->val)->py];
		if (((struct Track *)node->val)->owx >= 0) {
			((struct Track *)node->val)->owx -= offset[((struct Track *)node->val)->py];
		}
		node = node->next;
	}
}

/** @brief Set continuity detection region.
 ** @param bevtrack bird's eye view of track.
 ** @param wcdr width of continuity detection region.
 ** @param hcdr height of continuity detection region.
 ** @param cdreg continuity detection region.
 ** @param nlcdr number of left continuity detection region.
 ** @param nrcdr number of right continuity detection region.
 **/
void SetContinuityDetectRegion(List *bevtrack, int wcdr, int hcdr,
                               ObstacleDetectRegion *cdreg, int *nlcdr, int *nrcdr)
{	
	// Set continuity detection region of left track.
	int nodr = 0;
	Node *head = bevtrack->head;
	if (!head) {
		*nlcdr = 0;
		*nrcdr = 0;
		return;
	}
	
	if (0 == wcdr) {
		wcdr = ((((struct Track *)head->val)->px - ((struct Track *)head->val)->owx) << 1);
	}
	
	int bottomy = ((struct Track *)head->val)->py;
	while (head && 0 == ((struct Track *)head->val)->side) {
		if (((struct Track *)head->val)->py + hcdr > bottomy + 1) {
			head = head->next;
			continue;
		}
		
		cdreg[nodr].leftx = ((struct Track *)head->val)->px - (wcdr >> 1);
		cdreg[nodr].topy = ((struct Track *)head->val)->py;
		cdreg[nodr].width = wcdr;
		cdreg[nodr].height = hcdr;
		
		bottomy = cdreg[nodr].topy;
		nodr++;
	}
	
	*nlcdr = nodr;
	
	// Set continuity detection region of right track.
	if (!head) {
		*nrcdr = 0;
		return;
	}
	
	if (0 == wcdr) {
		wcdr = ((((struct Track *)head->val)->owx - ((struct Track *)head->val)->px) << 1);
	}
	
	bottomy = ((struct Track *)head->val)->py;
	while (head) {
		if (((struct Track *)head->val)->py + hcdr > bottomy + 1) {
			head = head->next;
			continue;
		}
		
		cdreg[nodr].leftx = ((struct Track *)head->val)->px - (wcdr >> 1);
		cdreg[nodr].topy = ((struct Track *)head->val)->py;
		cdreg[nodr].width = wcdr;
		cdreg[nodr].height = hcdr;
		
		bottomy = cdreg[nodr].topy;
		nodr++;
	}
	
	*nrcdr = nodr - *nlcdr;
}

/** @brief Calculate difference in brightness between neighboring regions.
 ** @param hshimg horizontal shifted bird's eye view image.
 ** @param width image width.
 ** @param height image height.
 ** @param cdreg continuity detection region.
 ** @param nlcdr number of left continuity detection region.
 ** @param nrcdr number of right continuity detection region.
 **/
void CalDiffOfBright(unsigned short *hshimg, int width, int height,
                     ObstacleDetectRegion *cdreg, int nlcdr, int nrcdr)
{
	cv::Mat _hshimg = cv::Mat(height, width, CV_16UC1, hshimg);
	
	int start[] = {0, nlcdr};
	int total[] = {nlcdr, nrcdr};
	for (int k = 0; k < 2; k++) {
		for (int i = start[k], j = start[k] + 1; i < start[k] + total[k] - 1; i++, j++) {
			cv::Mat nregion = _hshimg(cv::Rect(cdreg[i].leftx, cdreg[i].topy, cdreg[i].width, cdreg[i].height));
			cv::Mat fregion = _hshimg(cv::Rect(cdreg[j].leftx, cdreg[j].topy, cdreg[j].width, cdreg[j].height));
			cdreg[i].dbright = (float)cv::sum(cv::abs(nregion - fregion))[0];
		}
		
		float mean = 0;
		for (int i = start[k]; i < start[k] + total[k] - 1; i++) {
			mean += cdreg[i].dbright;
		}
		
		mean /= (total[k] - 1);
		float stdev = 0;
		for (int i = start[k]; i < start[k] + total[k] - 1; i++) {
			stdev += (cdreg[i].dbright - mean) * (cdreg[i].dbright - mean);
		}
		
		stdev = sqrt(stdev / (total[k] - 1));
		for (int i = start[k]; i < start[k] + total[k] - 1; i++) {
			cdreg[i].dbright = abs((cdreg[i].dbright - mean) / stdev);
		}
	}
}

/** @brief Set texture detection region.
 ** @param bevtrack bird's eye view of track.
 ** @param wtdr width of texture detection region.
 ** @param htdr height of texture detection region.
 ** @param tdreg texture detection region.
 ** @param ntdr number of texture detection region.
 **/
void SetTextureDetectRegion(List *bevtrack, int wtdr, int htdr,
                            ObstacleDetectRegion *tdreg, int *ntdr)
{
	// Find the start position of right track.
	Node *rightrack = bevtrack->head;
	while (rightrack && 0 == ((struct Track *)rightrack->val)->side) {
		rightrack = rightrack->next;
	}
	
	int nodr = 0;
	Node *leftrack = bevtrack->head;
	if (!leftrack || !rightrack) {
		*ntdr = 0;
		return;
	}
	
	if (0 == wtdr) {
		wtdr = ((struct Track *)rightrack->val)->px - ((struct Track *)leftrack->val)->px;
	}
	
	int bottomy = ((struct Track *)leftrack->val)->py;
	while (1) {
		while (leftrack && 0 == ((struct Track *)leftrack->val)->side && ((struct Track *)leftrack->val)->py + htdr > bottomy + 1) {
			leftrack = leftrack->next;
		}
		
		if (!leftrack || 1 == ((struct Track *)leftrack->val)->side) {
			break;
		}
		
		while (rightrack && ((struct Track *)rightrack->val)->py + htdr > bottomy + 1) {
			rightrack = rightrack->next;
		}
		
		if (!rightrack) {
			break;
		}
		
		tdreg[nodr].leftx = (((struct Track *)rightrack->val)->px + ((struct Track *)leftrack->val)->px - wtdr) >> 1;
		tdreg[nodr].topy = ((struct Track *)leftrack->val)->py;
		tdreg[nodr].width = wtdr;
		tdreg[nodr].height = htdr;
		
		bottomy = tdreg[nodr].topy;
		nodr++;
	}
	
	*ntdr = nodr;
}

/** @brief Calculate difference in variance between neighboring regions.
 ** @param hshimg horizontal shifted bird's eye view image.
 ** @param width image width.
 ** @param height image height.
 ** @param tdreg texture detection region.
 ** @param ntdr number of texture detection region.
 **/
void CalDiffOfVariance(unsigned short *hshimg, int width, int height,
                       ObstacleDetectRegion *tdreg, int ntdr)
{
	cv::Mat _hshimg = cv::Mat(height, width, CV_16UC1, hshimg);
	for (int i = 0, j = 1; i < ntdr - 1; i++, j++) {
		cv::Mat nregion = _hshimg(cv::Rect(tdreg[i].leftx, tdreg[i].topy, tdreg[i].width, tdreg[i].height));
		cv::Scalar nmean, nstdev;
		cv::meanStdDev(nregion, nmean, nstdev);

		cv::Mat fregion = _hshimg(cv::Rect(tdreg[j].leftx, tdreg[j].topy, tdreg[j].width, tdreg[j].height));	
		cv::Scalar fmean, fstdev;
		cv::meanStdDev(fregion, fmean, fstdev);
		
		tdreg[i].tex.var = (float)fabs(nstdev.val[0] * nstdev.val[0] - fstdev.val[0] * fstdev.val[0]);
	}
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

/** @brief Calculate track correlation between neighboring regions.
 ** @param hshimg horizontal shifted bird's eye view image.
 ** @param width image width.
 ** @param height image height.
 ** @param tdreg texture detection region.
 ** @param ntdr number of texture detection region.
 **/							 
void CalCorrelation(float *hshimg, int width, int height,
                    ObstacleDetectRegion *tdreg, int ntdr)
{
	cv::Mat _hshimg = cv::Mat(height, width, CV_32FC1, hshimg);
	for (int i = 0, j = 1; i < ntdr - 1; i++, j++) {
		cv::Mat nregion = _hshimg(cv::Rect(tdreg[i].leftx, tdreg[i].topy, tdreg[i].width, tdreg[i].height));
		cv::Mat fregion = _hshimg(cv::Rect(tdreg[j].leftx, tdreg[j].topy, tdreg[j].width, tdreg[j].height));
		
		cv::Mat corr;
		cv::Scalar nm = cv::mean(nregion);
		cv::Scalar fm = cv::mean(fregion);
		cv::matchTemplate(nregion - nm, fregion - fm, corr, CV_TM_CCORR_NORMED);
		tdreg[i].tex.corr = corr.ptr<float>(0)[0];
	}
}

/** @brief Track continuity discrimination.
 ** @param cdreg continuity detection region.
 ** @param nlcdr number of left continuity detection region.
 ** @param nrcdr number of right continuity detection region.
 ** @param camera camera model.
 ** @param offset horizontal offset table.
 ** @param dbrth difference threshold in brightness between neighboring regions.
 ** @param alarmlist obstacle alarm list.
 **/
void ContinuityDiscrimination(ObstacleDetectRegion *cdreg, int nlcdr, int nrcdr,
                              Camera *camera, int *offset, float dbrth, List *alarmlist)
{
	int start[] = {0, nlcdr};
	int total[] = {nlcdr, nrcdr};
	for (int i = 0; i < 2; i++) {
		for (int j = start[i]; j < start[i] + total[i] - 1; j++) {
			float score = -1;
			// Normalize score to [0,1].
			if (cdreg[j].dbright > dbrth) {
				score = ((cdreg[j].dbright > 3) ? 1 : cdreg[j].dbright / 3);
			}
			
			if (score < 0) {
				continue;
			}
			
			int sx[4] = {cdreg[j].leftx, cdreg[j].leftx, cdreg[j].leftx + cdreg[j].width - 1,
				cdreg[j].leftx + cdreg[j].width - 1};
			int sy[4] = {cdreg[j].topy, cdreg[j].topy + cdreg[j].height - 1, cdreg[j].topy,
				cdreg[j].topy + cdreg[j].height - 1};
			int dx[4];
			int dy[4];
			
			// Quad inverse horizontal offset and perspective transform.
			for (int k = 0; k < 4; k++) {
				sx[k] += offset[sy[k]];
				CameraPerspectiveTransformPerPoint(camera, sx[k], sy[k], &dx[k], &dy[k], 0);
			}
					
			int top = ODMin(dy[0], dy[2]);
			int left = ODMin(dx[0], dx[1]);
			int bottom = ODMax(dy[1], dy[3]);
			int right = ODMax(dx[2], dx[3]);
			float distance = 0;
			
			struct Alarm *alarm = (struct Alarm *)ListAlloc(sizeof(struct Alarm));
			if (!alarm) exit(-1);
			
			alarm->type = RAILWAY_ALARM_TRACK_OBSTACLE;
			alarm->top = top;
			alarm->left = left;
			alarm->bottom = bottom;
			alarm->right = right;
			alarm->isdangerous = 1;
			alarm->score = score;
			alarm->distance = distance;
			alarm->time = time(NULL);
			
			ListAddTail(alarmlist, alarm);
		}
	}
}

/** @brief Texture continuity discrimination.
 ** @param tdreg texture detection region.
 ** @param ntdr number of texture detection region.
 ** @param camera camera model.
 ** @param offset horizontal offset table.
 ** @param varth difference threshold of the variance of two consecutive regions.
 ** @param corrth correlation threshold of the two consecutive regions.
 ** @param alarmlist obstacle alarm list.
 **/
void TextureDiscrimination(ObstacleDetectRegion *tdreg, int ntdr, Camera *camera,
                           int *offset, float varth, float corrth, List *alarmlist)
{	
	for (int i = 0; i < ntdr - 1; i++) {
		float score = -1;
		// Normalized score to [0,1].
		if (tdreg[i].tex.var > varth) {
			score = ((tdreg[i].tex.var > 10000) ? 1 : (tdreg[i].tex.var / 10000));
		} else if (tdreg[i].tex.corr < corrth) {
			score = ((tdreg[i].tex.corr < 0) ? 1 : (1 - tdreg[i].tex.corr));
		}
		
		if (score < 0) {
			continue;
		}
		
		int sx[4] = {tdreg[i].leftx, tdreg[i].leftx, tdreg[i].leftx + tdreg[i].width - 1,
			tdreg[i].leftx + tdreg[i].width - 1};
		int sy[4] = {tdreg[i].topy, tdreg[i].topy + tdreg[i].height - 1, tdreg[i].topy,
			tdreg[i].topy + tdreg[i].height - 1};
		int dx[4];
		int dy[4];
		
		// Quad inverse horizontal offset and perspective transform.
		for (int j = 0; j < 4; j++) {
			sx[j] += offset[sy[j]];
			CameraPerspectiveTransformPerPoint(camera, sx[j], sy[j], &dx[j], &dy[j], 0);
		}
				
		int top = ODMin(dy[0], dy[2]);
		int left = ODMin(dx[0], dx[1]);
		int bottom = ODMax(dy[1], dy[3]);
		int right = ODMax(dx[2], dx[3]);
		float distance = 0;
			
		struct Alarm *alarm = (struct Alarm *)ListAlloc(sizeof(struct Alarm));
		if (!alarm) exit(-1);
		
		alarm->type = RAILWAY_ALARM_TRACK_OBSTACLE;
		alarm->top = top;
		alarm->left = left;
		alarm->bottom = bottom;
		alarm->right = right;
		alarm->isdangerous = 1;
		alarm->score = score;
		alarm->distance = distance;
		alarm->time = time(NULL);
		
		ListAddTail(alarmlist, alarm);
	}
}
									 
/** @brief View obstacle alarm line.
 ** @param self obstacle detector instance.
 **/
void ViewObstacleAlarmLine(ObstacleDetector *self)
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
		if (((struct Track *)node->val)->owx >= 0 && ((struct Track *)node->val)->owx < self->camera.xreso) {
			bgr.ptr<cv::Vec3b>(((struct Track *)node->val)->py)[((struct Track *)node->val)->owx] = cv::Vec3b(0, 255, 255);
		}
		if (((struct Track *)node->val)->mx >= 0 && ((struct Track *)node->val)->mx < self->camera.xreso) {
			bgr.ptr<cv::Vec3b>(((struct Track *)node->val)->py)[((struct Track *)node->val)->mx] = cv::Vec3b(0, 0, 255);
		}
		node = node->next;
	}
	
	List *alarmlist = (List *)self->alarmlist;
	Node *alarmnode = alarmlist->head;
	while (alarmnode) {
		struct Alarm *alarm = (struct Alarm *)alarmnode->val;
		cv::Rect rect(alarm->left, alarm->top, alarm->right - alarm->left, alarm->bottom - alarm->top);
		cv::rectangle(bgr, rect, cv::Scalar(255, 0, 0));
		alarmnode = alarmnode->next;
	}
		
	char path[128];
	static int timer = 0;
	sprintf(path, "obsalarmline\\%03d.png", timer++);
	cv::imwrite(path, bgr);
}

/** @brief Write bird's eye view image as file.
 ** @param self obstacle detector instance.
 **/
void ViewBEVImage(ObstacleDetector *self)
{
	cv::Mat _bevimg(self->camera.yreso, self->camera.xreso, CV_16UC1, self->hshimg);
	
	double minimum, maximum;
	cv::minMaxLoc(_bevimg, &minimum, &maximum);
	
	cv::Mat norm = 255 * (_bevimg - minimum) / (maximum - minimum);
	
	cv::Mat gray;
	norm.convertTo(gray, CV_8UC1);
	
	cv::Mat bgr;
	cv::cvtColor(gray, bgr, CV_GRAY2BGR);
	
	for (int i = 0; i < self->nlcdr + self->nrcdr; i++) {
		cv::Rect rect(self->cdreg[i].leftx, self->cdreg[i].topy, self->cdreg[i].width, self->cdreg[i].height);
		int thickness = 1;
		if (i != self->nlcdr - 1 && i != self->nlcdr + self->nrcdr - 1 &&
			self->cdreg[i].dbright > self->dbrth) {
			rect.y -= self->cdreg[i + 1].height;
			rect.height += self->cdreg[i + 1].height;
			thickness = 3;
		}
		cv::rectangle(bgr, rect, cv::Scalar(0, 0, 255), thickness);
	}
	
	for (int i = 0; i < self->ntdr; i++) {
		cv::Rect rect(self->tdreg[i].leftx, self->tdreg[i].topy, self->tdreg[i].width, self->tdreg[i].height);
		int thickness = 1;
		if (i < self->ntdr - 1 && (self->tdreg[i].tex.var > self->varth ||
			self->tdreg[i].tex.corr < self->corrth)) {
			rect.y -= self->tdreg[i + 1].height;
			rect.height += self->tdreg[i + 1].height;
			thickness = 3;
		}
		cv::rectangle(bgr, rect, cv::Scalar(255, 0, 0), thickness);
	}
	
	float minvar = 999999;
	float maxcorr = -1;

	for (int i = 0; i < self->ntdr - 1; i++) {		
		if (self->tdreg[i].tex.var < minvar) {
			minvar = self->tdreg[i].tex.var;
		}
		
		if (self->tdreg[i].tex.corr > maxcorr) {
			maxcorr = self->tdreg[i].tex.corr;
		}
	}
	
	int barmaxH = 100;
	cv::Point RA(barmaxH, self->tdreg[self->ntdr - 1].topy);
	cv::Point RB(barmaxH, self->tdreg[0].topy);
	cv::line(bgr, RA, RB, cv::Scalar(0, 0, 0));
	
	cv::Point RC(bgr.cols - 1 - barmaxH, self->tdreg[self->ntdr - 1].topy);
	cv::Point RD(bgr.cols - 1 - barmaxH, self->tdreg[0].topy);
	cv::line(bgr, RC, RD, cv::Scalar(0, 0, 0));
	
	float varul  = 10000;
	float corrlm = 0;
	
	for (int i = 0; i < self->ntdr - 1; i++) {
		float var = (self->tdreg[i].tex.var < varul ? self->tdreg[i].tex.var : varul);
		int x = (int)(barmaxH * (var - minvar) / (varul - minvar));
		cv::Point A(0, self->tdreg[i].topy);
		cv::Point B(x, self->tdreg[i].topy);
		cv::line(bgr, A, B, cv::Scalar(255, 255, 255));

		float corr = (self->tdreg[i].tex.corr > corrlm ? self->tdreg[i].tex.corr : corrlm);
		x = (int)(barmaxH * (corr - corrlm) / (maxcorr - corrlm));
		cv::Point C(self->camera.xreso - 1, self->tdreg[i].topy);
		cv::Point D(self->camera.xreso - 1 - x, self->tdreg[i].topy);
		cv::line(bgr, C, D, cv::Scalar(255, 255, 255));
	}

	Node *node = self->bevtracklist.head;
	while (node) {
		if (((struct Track *)node->val)->px >= 0 && ((struct Track *)node->val)->px < self->camera.xreso &&
			((struct Track *)node->val)->py >= 0 && ((struct Track *)node->val)->py < self->camera.yreso) {
			bgr.ptr<cv::Vec3b>(((struct Track *)node->val)->py)[((struct Track *)node->val)->px] = cv::Vec3b(0, 255, 255);
			if (((struct Track *)node->val)->owx >= 0 && ((struct Track *)node->val)->owx < self->camera.xreso) {
				bgr.ptr<cv::Vec3b>(((struct Track *)node->val)->py)[((struct Track *)node->val)->owx] = cv::Vec3b(255, 255, 255);
			}
		}
		node = node->next;
	}
		
	char path[128];
	static int timer = 0;
	sprintf(path, "bev\\%03d.png", timer++);
	cv::imwrite(path, bgr);
}