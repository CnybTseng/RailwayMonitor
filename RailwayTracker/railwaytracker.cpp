/** @file railwaytracker.c - Implementation
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/stat.h>
#include <windows.h>

#include "opencv2/opencv.hpp"
#include "json/json.h"
#include "pthread.h"
#include "fifo.h"
#include "list.h"
#include "scanlineparam.h"
#include "railwaytracker.h"

#define RING_IMAGE_NUM					4

/** @typedef struct ScanLine.
 ** @brief measurement scan line data structure.
 **/
typedef struct
{
	int x;				/*< x of center of scan line. */
	int y;				/*< y of center of scan line. */
	int radius;			/*< scan line radius. */
	int bx;				/*< x of extracted lane border point. */
	int px;				/*< x of state posteriori projection. */
	float dir;			/*< direction of projected lane boundary at (x,y). */
	float edir;			/*< matching edge point direction. */
} ScanLine;

/** @typedef struct LaneStateModel.
 ** @brief Lane state model data structure.
 **/
typedef struct
{
	double phi1;		/*< yaw angle. */
	double phi2;		/*< pitch angle. */
	double y0;			/*< lateral offset. */
	double b;			/*< lane width. */
	double ch0;			/*< horizontal curvature. */
	double ch1;			/*< alteration of horizontal curvature. */
	double cv0;			/*< vertical curvature. */
} LaneStateModel;

struct tagRailwayTracker
{
	Fifo *rawring;						/*< raw image ring buffer. */
	unsigned short *rawimg;				/*< raw image. */
	unsigned short *blurimg;			/*< blurred image. */
	float *magximg;						/*< x-gradient magnitude image. */
	float *magyimg;						/*< y-gradient magnitude image. */
	float *magimg;						/*< gradient magnitude image. */
	float *dirimg;						/*< gradient direction image. */
	Fifo *trackring;					/*< track ring buffer. */
	char *tracklist;					/*< track list for railway monitor. */
	char *tracklist1;					/*< track list for obstacle detector. */
	char *tracklist2;					/*< track list for pedestrian detector. */
	Camera camera;						/*< camera set. */
	PedestrianDetector *pdetector;		/*< pedestrian detector. */
	ObstacleDetector *odetector;		/*< obstacle detector. */
	float rslad;						/*< ratio of selected look ahead distance. */
	int nvd;							/*< number of valid look ahead distance. */
	int numscaninter;					/*< number of scan lines. */
	int minscaninter;					/*< minimum scan line interval. */
	int maxsumscaninter;				/*< maximum sum of scan line intervals. */
	int sumscaninter;					/*< sum of scan line intervals. */
	int maxscaninter;					/*< maximum scan line interval. */
	int maxscanrad;						/*< maximum scan line radius */
	float scalei;						/*< scan line interval scale factor. */
	float scaler;						/*< scan line radius scale factor. */
	ScanLine *scanline;					/*< scan line. */
	int nlsl;							/*< number of left scan lines. */
	int nrsl;							/*< number of right scan lines. */
	LaneStateModel s0;					/*< initial state. */
	float *ladist;						/*< look ahead distance. */
	int dynadim;						/*< dynamic vector dimension. */
	int measdim;						/*< measurement vector dimension. */
	double *measurement;				/*< measurement vector. */
	double *prediction;					/*< prediction vector. */
	double *estimation;					/*< estimation vector. */
	float goodfithresh;					/*< goodness fit threshold. */
	float pedwarnwidth;					/*< pedestrian warning region width. */
	float obswarnwidth;					/*< obstacle warning region width. */
	float lateraloffsetlowlim;			/*< lateral offset lower limit. */
	float lateraloffsethiglim;			/*< lateral offset higher limit. */
	float directionbiasthresh;			/*< predicted and measured direction bias threshold. */
	int leftstraightrackceil;			/*< left straight track position. */
	int rightstraightrackfloor;			/*< right straight track position. */
	pthread_t tid;						/*< thread ID of tracking thread. */
	pthread_t tidconf;					/*< thread ID of configuration update thread. */
	int disable;						/*< disable railway tracking function. */
	int runswitch;						/*< tracking thread run switch. */
};

/** @name Local functions.
 ** @{ */
static unsigned int RoundupPowerOf2(unsigned int a);
static int InitScanLineParam(const char *path, float *rslad, int *nsli, int *minsi,
                             int *mssi, int *maxsi, int *msr, float *si, float *sr);
static int InitGoodnessOfFitThresh(const char *path, float *goodfithresh);
static int InitPedWarnRegionWidth(const char *path, float *warnwidth);
static int InitObsWarnRegionWidth(const char *path, float *warnwidth);
static int InitLaneState(const char *path, LaneStateModel *lsm);
static int InitProcessNoiseCov(const char *path, double *pnc, int nels);
static int InitMeasurementNoiseCov(const char *path, double *mnc, int nels);
static int InitErrorCovPost(const char *path, double *ecp, int nels);
static int InitLateralOffsetLimit(const char *path, float *lateraloffsetlowlim, float *lateraloffsethiglim);
static int InitDirectionBiasThreshold(const char *path, float *bias_thresh);
static int InitStraightTrackPosition(const char *path, int *leftstraightrackceil, int *rightstraightrackfloor);
static void InitExtendedKalmanFilter(RailwayTracker *self, double *sp, double *ecp,
                                     double *jmm, int dynadim, int measdim);
static void InitScanLine(ScanLine *sl, int nsl);
static int CaLoodaheaDist(Camera *camera, LaneStateModel *model, float *dist, int ndist);
static void ChooseValidLookahead(Camera *camera, LaneStateModel *model, float *dist,
                                 int ndist, int *ylfs, int *nlvd, int *yrfs, int *nrvd);
static int CalScanLine(int nsi, int msi, float si, float sr, float *lad, int nvd, Camera *cam,
                       int msr, LaneStateModel *lsm, int side, int yfs, ScanLine *sl);
static float CaLaneDirection(Camera *cam, LaneStateModel *lsm, float dist, int side);
static void CalMeasureJacoMatrix(Camera *cam, float *lad, ScanLine *sl, int nsl, int side,
                                 LaneStateModel *lsm, double *jm, int jmw, int jmh);
static void *RailwayTrackThread(void *param);
static void Blur(unsigned short *src, int width, int height, unsigned short *dst);
static void CalGradient(unsigned short *src, int width, int height, float *magx,
                        float *magy, float *mag, float *dir);
static void ExtractTrackBorder(float *mag, float *dir, int width, int height,
                               ScanLine *sl, int nsl);
static float GaussWeight(float sigma, int dist);
static float MatchPredictDirection(ScanLine *prev, ScanLine *curr, int x);
static int CheckGoodnessOfFit(ScanLine *sl, int nsl, float thresh);
static void CalResidualError(Camera *cam, float *lad, ScanLine *sl, int nsl, int side,
                             double *statepre, double *jmm, int jmw, int jmh);
static void GetMeasurement(ScanLine *sl, int nsl, double *measure);
static void State2Model(double *state, LaneStateModel *lsm);
static int SolveQuadraticEquation(double p1, double p2, double p3, double *sol);
static void ProjectStatePost(Camera *cam, LaneStateModel *lsm, ScanLine *sl, int nsl, int side);
static int SaveLaneState(const char *path, LaneStateModel *lsm);
static int SaveErrorCovPost(const char *path, double *ecp, int width, int height);
static void CalTrackBoundary(float *mag, float *dir, Camera *cam, ScanLine *sl, int nsl, int side,
                             float pww, float oww, LaneStateModel *lsm, List *tracklist);
static void *ConfigUpdateThread(void *param);
static void PrintMatrix(double *data, int width, int height, const char *head);
static void DrawTrackerScanLines(RailwayTracker *self, unsigned char *bkgrnd);
static int UpdateCutoffLine(ScanLine *sl, int nsl, float bias_thresh);
/** @} */

/** @brief Create a new instance of railway tracker.
 ** @param camera camera model.
 ** @param pdetector pedestrian detector.
 ** @param odetector obstacle detector.
 ** @return the new instance of railway tracker.
 **/
RailwayTracker *RailwayTrackerCreate(Camera *camera, PedestrianDetector *pdetector,
                                     ObstacleDetector *odetector)
{	
	RailwayTracker *self = (RailwayTracker *)malloc(sizeof(RailwayTracker));
	if (!self) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->rawring = NULL;
	self->rawimg = NULL;
	self->blurimg = NULL;
	self->magximg = NULL;
	self->magyimg = NULL;
	self->magimg = NULL;
	self->dirimg = NULL;
	self->trackring = NULL;
	self->tracklist = NULL;
	self->tracklist1 = NULL;
	self->tracklist2 = NULL;
	self->camera = *camera;
	self->pdetector = pdetector;
	self->odetector = odetector;
	self->rslad = 0.92f;
	self->numscaninter = 20;
	self->minscaninter = 4;
	self->maxsumscaninter = (int)(camera->yreso * 0.45f);
	self->sumscaninter = 0;
	self->maxscaninter = 24;
	self->maxscanrad = 64;
	self->scalei = 0.92f;
	self->scaler = 0.8f;
	self->scanline = NULL;
	self->nlsl = 0;
	self->nrsl = 0;
	self->ladist = NULL;
	self->dynadim = 7;	
	self->measdim = 0;
	self->measurement = NULL;
	self->prediction = NULL;
	self->estimation = NULL;
	self->goodfithresh = 10;
	self->pedwarnwidth = 4.88f;
	self->obswarnwidth = 2.435f;
	self->lateraloffsetlowlim = -0.5f;
	self->lateraloffsethiglim = 0.5f;
	self->directionbiasthresh = 0.866f;
	self->leftstraightrackceil = camera->xreso - 1;
	self->rightstraightrackfloor = 0;
	self->disable = 0;
	self->runswitch = 0;
	
	InitScanLineParam("RailwayMonitor.json", &self->rslad, &self->numscaninter, &self->minscaninter,
		&self->maxsumscaninter, &self->maxscaninter, &self->maxscanrad, &self->scalei, &self->scaler);
	InitGoodnessOfFitThresh("RailwayMonitor.json", &self->goodfithresh);
	InitPedWarnRegionWidth("RailwayMonitor.json", &self->pedwarnwidth);
	InitObsWarnRegionWidth("RailwayMonitor.json", &self->obswarnwidth);
	InitLateralOffsetLimit("RailwayMonitor.json", &self->lateraloffsetlowlim, &self->lateraloffsethiglim);
	InitDirectionBiasThreshold("RailwayMonitor.json", &self->directionbiasthresh);
	InitStraightTrackPosition("RailwayMonitor.json", &self->leftstraightrackceil, &self->rightstraightrackfloor);
	CalScanLineParam_initialize();
	
	if (InitLaneState("RailwayMonitor.json", &self->s0)) {
		goto clean;
	}
	
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
	
	self->blurimg = (unsigned short *)malloc(camera->yreso * camera->xreso * sizeof(unsigned short));
	if (!self->blurimg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->magximg = (float *)malloc(camera->yreso * camera->xreso * sizeof(float));
	if (!self->magximg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->magyimg = (float *)malloc(camera->yreso * camera->xreso * sizeof(float));
	if (!self->magyimg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->magimg = (float *)malloc(camera->yreso * camera->xreso * sizeof(float));
	if (!self->magimg) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->dirimg = (float *)malloc(camera->yreso * camera->xreso * sizeof(float));
	if (!self->dirimg) {
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
	
	self->tracklist1 = (char *)malloc(RoundupPowerOf2(sizeof(List)));
	if (!self->tracklist1) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->tracklist2 = (char *)malloc(RoundupPowerOf2(sizeof(List)));
	if (!self->tracklist2) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->scanline = (ScanLine *)malloc((camera->yreso << 1) * sizeof(ScanLine));
	if (!self->scanline) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->ladist = (float *)malloc(camera->yreso * sizeof(float));
	if (!self->ladist) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	// Calculate maximum number of valid look ahead distances.
	self->nvd = CaLoodaheaDist(camera, &self->s0, self->ladist, camera->yreso);
	self->sumscaninter = (int)(self->nvd * self->rslad);
	
	// Calculate scan line parameters.
	float _maxscaninter = 1;
	CalScanLineParam((float)self->numscaninter, (float)self->minscaninter, (float)self->sumscaninter,
		&_maxscaninter, &self->scalei);
	self->maxscaninter = (int)_maxscaninter;
		
	// Calculate scan lines corresponding to the state model.
	self->nlsl = CalScanLine(self->numscaninter, self->maxscaninter, self->scalei, self->scaler, self->ladist,
		self->nvd, &self->camera, self->maxscanrad, &self->s0, -1, camera->yreso - 1, self->scanline);
	self->nrsl = CalScanLine(self->numscaninter, self->maxscaninter, self->scalei, self->scaler, self->ladist,
		self->nvd, &self->camera, self->maxscanrad, &self->s0,  1, camera->yreso - 1, self->scanline + self->nlsl);
	
	// Preset dimension of measurement vector.
	self->measdim = (self->numscaninter + 1) << 1;	
		
	self->measurement = (double *)malloc(self->measdim * sizeof(double));
	if (!self->measurement) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->prediction = (double *)malloc(self->measdim * sizeof(double));
	if (!self->prediction) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		goto clean;
	}
	
	self->estimation = (double *)malloc(self->measdim * sizeof(double));
	if (!self->estimation) {
		fprintf(stderr, "malloc fail[%s:%d].\n", __FILE__, __LINE__);
		clean:RailwayTrackerDestroy(self);
	}
	
	return self;
}

/** @brief Put raw image data in ring buffer.
 **        Be careful: len=2^n.
 ** @param self railway tracker instance.
 ** @param data raw image data.
 ** @param len raw image data length.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int RailwayTrackerPutData(RailwayTracker *self, unsigned char *data, int len)
{
	int wrt = fifo_put(self->rawring, (const char *)data, len);
	if (wrt != len) {
		return -1;
	}
	
	return 0;
}

/** @brief Begin railway tracking thread.
 ** @param self railway tracker instance.
 ** @return  -1 if fail,
 **           0 if success,
 **           1 if the thread has been started.
 **/
int RailwayTrackerBegin(RailwayTracker *self)
{
	if (self->runswitch) {
		return 1;
	}

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	int ret = pthread_create(&self->tid, &attr, RailwayTrackThread, self);
	if (0 != ret) {
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	// ret = pthread_create(&self->tidconf, &attr, ConfigUpdateThread, self);
	// if (0 != ret) {
	// 	fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
	// 	return -1;
	// }
	
	self->runswitch = 1;
	pthread_attr_destroy(&attr);
	
	return 0;
}

/** @brief Get track from ring buffer.
 ** @param self railway tracker instance.
 ** @param track track list.
 ** @param len length of track list.
 ** @return FIFO read length.
 **/
int RailwayTrackerGetData(RailwayTracker *self, char *track, int len)
{
	return fifo_get(self->trackring, track, len);
}

/** @brief Switch railway tracker on of off.
 ** @param self railway tracker instance.
 ** @param onoff on(=1) or off(=0)
 **/
void RailwayTrackerOnOff(RailwayTracker *self, int onoff)
{
	self->disable = onoff ^ 0x01;
}

/** @brief Kill the railway tracking thread.
 ** @param self railway tracker instance.
 **/
void RailwayTrackerEnd(RailwayTracker *self)
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
		
		// ret = pthread_kill(self->tidconf, 0);
		// if (ESRCH == ret) {
		// 	fprintf(stderr, "the thread didn't exists or already quit[%s:%d].\n", __FILE__, __LINE__);
		// 	return;
		// } else if (EINVAL == ret) {
		// 	fprintf(stderr, "signal is invalid[%s:%d].\n", __FILE__, __LINE__);
		// 	return;
		// } else {
		// 	continue;
		// }
	}
}

/** @brief Release all resources of railway tracker.
 ** @param self railway tracker instance.
 **/
void RailwayTrackerDestroy(RailwayTracker *self)
{
	if (self) {
		if (self->rawring) {
			fifo_delete(self->rawring);
		}
		
		if (self->rawimg) {
			free(self->rawimg);
			self->rawimg = NULL;
		}
		
		if (self->blurimg) {
			free(self->blurimg);
			self->blurimg = NULL;
		}
		
		if (self->magximg) {
			free(self->magximg);
			self->magximg = NULL;
		}
		
		if (self->magyimg) {
			free(self->magyimg);
			self->magyimg = NULL;
		}
		
		if (self->magimg) {
			free(self->magimg);
			self->magimg = NULL;
		}
		
		if (self->dirimg) {
			free(self->dirimg);
			self->dirimg = NULL;
		}
		
		if (self->trackring) {
			fifo_delete(self->trackring);
		}
		
		if (self->tracklist) {
			free(self->tracklist);
			self->tracklist = NULL;
		}
		
		if (self->tracklist1) {
			free(self->tracklist1);
			self->tracklist1 = NULL;
		}
		
		if (self->tracklist2) {
			free(self->tracklist2);
			self->tracklist2 = NULL;
		}
		
		if (self->scanline) {
			free(self->scanline);
			self->scanline = NULL;
		}
		
		if (self->ladist) {
			free(self->ladist);
			self->ladist = NULL;
		}
		
		if (self->measurement) {
			free(self->measurement);
			self->measurement = NULL;
		}
		
		if (self->prediction) {
			free(self->prediction);
			self->prediction = NULL;
		}
		
		if (self->estimation) {
			free(self->estimation);
			self->estimation = NULL;
		}
		
		CalScanLineParam_terminate();
		
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

/** @brief Initialize scan line parameters.
 ** @param path configuration file path.
 ** @param rslad ratio of selected look ahead distance.
 ** @param nsli number of scan line intervals.
 ** @param minsi minimum scan interval.
 ** @param mssi maximum sum of scan line intervals.
 ** @param maxsi maximum scan interval.
 ** @param msr maximum scan radius.
 ** @param si scan line interval scale factor.
 ** @param sr scan line radius scale factor.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitScanLineParam(const char *path, float *rslad, int *nsli, int *minsi,
                      int *mssi, int *maxsi, int *msr, float *si, float *sr)
{
	std::ifstream ifs(path, std::ios::binary);

	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["ratiOfSelectedLookaheadDistance"].isDouble()) {
			*rslad = root["ratiOfSelectedLookaheadDistance"].asFloat();
		} else {
			fprintf(stderr, "'ratiOfSelectedLookaheadDistance' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["numScanLineInter"].isInt()) {
			*nsli = root["numScanLineInter"].asInt();
		} else {
			fprintf(stderr, "'numScanLineInter' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["minScanLineInter"].isInt()) {
			*minsi = root["minScanLineInter"].asInt();
		} else {
			fprintf(stderr, "'minScanLineInter' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["maxSumScanLineInter"].isInt()) {
			*mssi = root["maxSumScanLineInter"].asInt();
		} else {
			fprintf(stderr, "'maxSumScanLineInter' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["maxScanLineInter"].isInt()) {
			*maxsi = root["maxScanLineInter"].asInt();
		} else {
			fprintf(stderr, "'maxScanLineInter' isn't interger[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["maxScanLineRadius"].isInt()) {
			*msr = root["maxScanLineRadius"].asInt();
		} else {
			fprintf(stderr, "'maxScanLineRadius' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["scaleInter"].isDouble()) {
			*si = root["scaleInter"].asFloat();
		} else {
			fprintf(stderr, "'scaleInter' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["scaleRadius"].isDouble()) {
			*sr = root["scaleRadius"].asFloat();
		} else {
			fprintf(stderr, "'scaleRadius' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize goodness of fit threshold.
 ** @param path configuration file path.
 ** @param goodfithresh goodness of fit threshold.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitGoodnessOfFitThresh(const char *path, float *goodfithresh)
{
	std::ifstream ifs(path, std::ios::binary);

	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["goodFitThresh"].isDouble()) {
			*goodfithresh = root["goodFitThresh"].asFloat();
		} else {
			fprintf(stderr, "'goodFitThresh' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize pedestrian warning region width.
 ** @param path configuration file path.
 ** @param warnwidth warning region width.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitPedWarnRegionWidth(const char *path, float *warnwidth)
{
	std::ifstream ifs(path, std::ios::binary);

	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["pedWarnRegionWidth"].isDouble()) {
			*warnwidth = root["pedWarnRegionWidth"].asFloat();
		} else {
			fprintf(stderr, "'pedWarnRegionWidth' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize obstacle warning region width.
 ** @param path configuration file path.
 ** @param warnwidth warning region width.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitObsWarnRegionWidth(const char *path, float *warnwidth)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["obsWarnRegionWidth"].isDouble()) {
			*warnwidth = root["obsWarnRegionWidth"].asFloat();
		} else {
			fprintf(stderr, "'obsWarnRegionWidth' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize railway lane state.
 ** @param path configuration file path.
 ** @param lsm lane state model.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitLaneState(const char *path, LaneStateModel *lsm)
{
	// Load from last lane state backup.
	struct stat statbuf;
	if (-1 != stat("LaneState.txt", &statbuf)) {
		FILE *fp = fopen("LaneState.txt", "r");
		if (!fp) {
			fprintf(stderr, "fopen fail[%s:%d].\n", __FILE__, __LINE__);
		} else {
			fscanf(fp, "%lf\n", &lsm->phi1);
			fscanf(fp, "%lf\n", &lsm->phi2);
			fscanf(fp, "%lf\n", &lsm->y0);
			fscanf(fp, "%lf\n", &lsm->b);
			fscanf(fp, "%lf\n", &lsm->ch0);
			fscanf(fp, "%lf\n", &lsm->ch1);
			fscanf(fp, "%lf\n", &lsm->cv0);
			fclose(fp);
			return 0;
		}
	} else {
		fprintf(stderr, "stat error[%s:%d].\n", __FILE__, __LINE__);
	}

	// Load default lane state.
	std::ifstream ifs(path, std::ios::binary);

	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["laneInitState"]["yawAngle"].isDouble()) {
			lsm->phi1 = root["laneInitState"]["yawAngle"].asDouble();
		} else {
			fprintf(stderr, "'yawAngle' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["pitchAngle"].isDouble()) {
			lsm->phi2 = root["laneInitState"]["pitchAngle"].asDouble();
		} else {
			fprintf(stderr, "'pitchAngle' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["lateralOffset"].isDouble()) {
			lsm->y0 = root["laneInitState"]["lateralOffset"].asDouble();
		} else {
			fprintf(stderr, "'lateralOffset' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["laneWidth"].isDouble()) {
			lsm->b = root["laneInitState"]["laneWidth"].asDouble();
		} else {
			fprintf(stderr, "'laneWidth' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["horizontalCurvature"].isDouble()) {
			lsm->ch0 = root["laneInitState"]["horizontalCurvature"].asDouble();
		} else {
			fprintf(stderr, "'horizontalCurvature' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["alterationOfHorizontalCurvature"].isDouble()) {
			lsm->ch1 = root["laneInitState"]["alterationOfHorizontalCurvature"].asDouble();
		} else {
			fprintf(stderr, "'alterationOfHorizontalCurvature' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["verticalCurvature"].isDouble()) {
			lsm->cv0 = root["laneInitState"]["verticalCurvature"].asDouble();
		} else {
			fprintf(stderr, "'verticalCurvature' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}

	return 0;
}

/** @brief Initialize process noise covariance from configuration.
 ** @param path configuration file path.
 ** @param pnc process noise covariance matrix.
 ** @param nels number of elements of process noise covariance matrix.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitProcessNoiseCov(const char *path, double *pnc, int nels)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["processNoiseCov"].isArray()) {
			for (size_t i = 0; i < root["processNoiseCov"].size(); i++) {
				pnc[i] = root["processNoiseCov"][i].asDouble();
			}
		} else {
			fprintf(stderr, "'processNoiseCov' isn't array[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}

	return 0;
}

/** @brief Initialize measurement noise covariance from configuration.
 ** @param path configuration file path.
 ** @param mnc measurement noise covariance matrix.
 ** @param nels number of elements of measurement noise covariance matrix.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitMeasurementNoiseCov(const char *path, double *mnc, int nels)
{
	std::ifstream ifs(path, std::ios::binary);
	
	int width = (int)sqrt(nels);
	int height = width;
	cv::Mat _mnc(height, width, CV_64FC1, mnc);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["measurementNoiseStdev"].isDouble()) {
			double mnsd = root["measurementNoiseStdev"].asDouble();
			cv::setIdentity(_mnc, cv::Scalar(mnsd * mnsd));
		} else {
			fprintf(stderr, "'measurementNoiseStdev' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize posteriori error estimate covariance from configuration.
 ** @param path configuration file path.
 ** @param ecp posteriori error estimate covariance matrix.
 ** @param nels number of elements of posteriori error estimate covariance matrix.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitErrorCovPost(const char *path, double *ecp, int nels)
{
	// Load from last error covariance posteriori backup.
	struct stat statbuf;
	if (-1 != stat("ErrorCovPost.txt", &statbuf)) {
		FILE *fp = fopen("ErrorCovPost.txt", "r");
		if (!fp) {
			fprintf(stderr, "fopen fail[%s:%d].\n", __FILE__, __LINE__);
		} else {
			int width = (int)sqrt(nels);
			int height = width;
			for (int y = 0; y < height; y++) {
				for (int x = 0; x < width; x++) {
					fscanf(fp, "%lf\n", &ecp[y * width + x]);
				}
			}
			
			fclose(fp);
			return 0;
		}
	} else {
		fprintf(stderr, "stat error[%s:%d].\n", __FILE__, __LINE__);
	}
	
	// Load default error covariance posteriori.
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["errorCovPost"].isArray()) {
			for (size_t i = 0; i < root["errorCovPost"].size(); i++) {
				ecp[i] = root["errorCovPost"][i].asDouble();
			}
		} else {
			fprintf(stderr, "'errorCovPost' isn't array[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}

	return 0;
}

/** @brief Initialize lateral offset limit.
 ** @param path configuration file path.
 ** @param lateraloffsetlowlim lateral offset lower limit.
 ** @param lateraloffsethiglim lateral offset higher limit.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitLateralOffsetLimit(const char *path, float *lateraloffsetlowlim, float *lateraloffsethiglim)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["lateralOffsetLowLim"].isDouble()) {
			*lateraloffsetlowlim = root["lateralOffsetLowLim"].asFloat();
		} else {
			fprintf(stderr, "'lateralOffsetLowLim' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["lateralOffsetHigLim"].isDouble()) {
			*lateraloffsethiglim = root["lateralOffsetHigLim"].asFloat();
		} else {
			fprintf(stderr, "'lateralOffsetHigLim' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}

	return 0;
}

/** @brief Initialize predicted and measured direction bias threshold.
 ** @param path configuration file path.
 ** @param bias_thresh predicted and measured direction bias threshold.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitDirectionBiasThreshold(const char *path, float *bias_thresh)
{
	std::ifstream ifs(path, std::ios::binary);
	
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (root["DirectionBiasThresh"].isDouble()) {
			*bias_thresh = root["DirectionBiasThresh"].asFloat();
		} else {
			fprintf(stderr, "'DirectionBiasThresh' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize straight track positions.
 ** @param path configuration file path.
 ** @param leftstraightrackceil left straight track position ceil.
 ** @param rightstraightrackfloor right straight track position floor.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int InitStraightTrackPosition(const char *path, int *leftstraightrackceil, int *rightstraightrackfloor)
{
	std::ifstream ifs(path, std::ios::binary);

	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {		
		if (root["leftStraightTrackCeil"].isInt()) {
			*leftstraightrackceil = root["leftStraightTrackCeil"].asInt();
		} else {
			fprintf(stderr, "'leftStraightTrackCeil' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["rightStraightTrackFloor"].isInt()) {
			*rightstraightrackfloor = root["rightStraightTrackFloor"].asInt();
		} else {
			fprintf(stderr, "'rightStraightTrackFloor' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	} else {
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

/** @brief Initialize Extended Kalman Filter.
 ** @param self railway tracker instance.
 ** @param sp state posteriori.
 ** @param ecp error covariance posteriori.
 ** @param jmm Jacobian matrix of measurement function.
 ** @param dynadim dynamic dimension.
 ** @param measdim measurement dimension.
 **/
void InitExtendedKalmanFilter(RailwayTracker *self, double *sp, double *ecp,
                              double *jmm, int dynadim, int measdim)
{
	int nlsl = CalScanLine(self->numscaninter, self->maxscaninter, self->scalei, self->scaler, self->ladist,
		self->nvd, &self->camera, self->maxscanrad, &self->s0, -1, self->camera.yreso - 1, self->scanline);
	int nrsl = CalScanLine(self->numscaninter, self->maxscaninter, self->scalei, self->scaler, self->ladist,
		self->nvd, &self->camera, self->maxscanrad, &self->s0,  1, self->camera.yreso - 1, self->scanline + nlsl);
	
	// Initialize Jacobian matrix of measurement function.
	CalMeasureJacoMatrix(&self->camera, self->ladist, self->scanline,        nlsl, -1,
		&self->s0, jmm,                  dynadim, nlsl);
	CalMeasureJacoMatrix(&self->camera, self->ladist, self->scanline + nlsl, nrsl,  1,
		&self->s0, jmm + nlsl * dynadim, dynadim, nrsl);
	
	// Initialize posteriori error estimate covariance matrix.	(P)
	remove("ErrorCovPost.txt");
	InitErrorCovPost("RailwayMonitor.json", ecp, dynadim * dynadim);
	
	remove("LaneState.txt");
	
	// Initialize posteriori (corrected) state.	
	sp[0] = self->s0.phi1;
	sp[1] = self->s0.phi2;
	sp[2] = self->s0.y0;
	sp[3] = self->s0.b;
	sp[4] = self->s0.ch0;
	sp[5] = self->s0.ch1;
	sp[6] = self->s0.cv0;
}

/** @brief Initialize scan lines.
 ** @param sl scan lines.
 ** @param nsl number of scan lines.
 **/
void InitScanLine(ScanLine *sl, int nsl)
{
	for (int i = 0; i < nsl; i++) {
		sl[i].x = -1;
		sl[i].y = -1;
		sl[i].bx = -1;
		sl[i].px = -1;
	}
}

/** @brief Calculate look ahead distance.
 **        Solve equation like 'p1*x^2+p2*x+p3=0'.
 ** @param camera camera model.
 ** @param model lane state model.
 ** @param dist look ahead distance.
 ** @param ndist maximum number of look ahead distance.
 ** @return number of valid look ahead distance.
 **/
int CaLoodaheaDist(Camera *camera, LaneStateModel *model, float *dist, int ndist)
{
	double p1 = -camera->fy * model->cv0 / 2;
	double p3 = camera->fy * camera->h;
	int nvd = 0;
	
	for (int y = camera->yreso - 1; y >= 0; y--) {
		double p2 = camera->fy * model->phi2 + camera->cy - y;
		double sol;
		if (!SolveQuadraticEquation(p1, p2, p3, &sol) && sol > 0) {
			dist[nvd++] = (float)sol;
		} else {
			break;
		}
	}
	
	return nvd;
}

/** @brief Choose valid look ahead distance.
 ** @param camera camera model.
 ** @param model lane state model.
 ** @param dist look ahead distance.
 ** @param ndist number of valid look ahead distance.
 ** @param ylfs y position of the left first scan line.
 ** @param nlvd number of left valid look ahead distance.
 ** @param yrfs y position of the right first scan line.
 ** @param nrvd number of right valid look ahead distance.
 **/
void ChooseValidLookahead(Camera *camera, LaneStateModel *model, float *dist,
                          int ndist, int *ylfs, int *nlvd, int *yrfs, int *nrvd)
{
	int doxbnr =  5;
	int border = 20;
	int prevlxb;
	int prevrxb;
	*ylfs = -1;
	*nlvd =  0;
	*yrfs = -1;
	*nrvd =  0;
	
	int yend = camera->yreso - 1 - ndist;
	
	// Left valid look ahead distance.
	for (int y = camera->yreso - 1; y > yend; y--) {
		float d = dist[camera->yreso - 1 - y];
		double temp1 = model->ch0 * pow(d, 2) / 2 + model->ch1 * pow(d, 3) / 6;
		double temp2 = camera->fx * (-model->y0 + model->phi1 * d) / d + camera->cx;
		
		double yl = -model->b / 2 + temp1;
		int xb = (int)(camera->fx * yl / d + temp2 + 0.5f);
		if ((-1 == *ylfs) && (xb < border || xb > camera->xreso - 1 - border)) {
			continue;
		}
		
		if (-1 == *ylfs) {
			*ylfs = y;
		}
		
		if (xb < border || xb > camera->xreso - border || (y < *ylfs && abs(xb - prevlxb) > doxbnr)) {
			break;
		}
			
		prevlxb = xb;
		(*nlvd)++;
	}
	
	// Right valid look ahead distance.
	for (int y = camera->yreso - 1; y > yend; y--) {
		float d = dist[camera->yreso - 1 - y];
		double temp1 = model->ch0 * pow(d, 2) / 2 + model->ch1 * pow(d, 3) / 6;
		double temp2 = camera->fx * (-model->y0 + model->phi1 * d) / d + camera->cx;
		
		double yl = model->b / 2 + temp1;
		int xb = (int)(camera->fx * yl / d + temp2 + 0.5f);
		if ((-1 == *yrfs) && (xb < border || xb > camera->xreso - 1 - border)) {
			continue;
		}
		
		if (-1 == *yrfs) {
			*yrfs = y;
		}
		
		if (xb < border || xb > camera->xreso - border || (y < *yrfs && abs(xb - prevrxb) > doxbnr)) {
			break;
		}
			
		prevrxb = xb;
		(*nrvd)++;
	}
}

/** @brief Calculate scan lines.
 **        Scane line interval is a geometric progression with common ratio si.
 ** @param nsi number scan line intervals.
 ** @param msi maximum scan line interval.
 ** @param si scan line interval scale factor.
 ** @param sr scan line radius scale factor.
 ** @param lad look ahead distance.
 ** @param nvd number of valid look ahead distance.
 ** @param cam camera model.
 ** @param msr maximum scan line radius.
 ** @param lsm lane state model.
 ** @param side left(=-1) or right(=1)side.
 ** @param yfs y position of the first scan line.
 ** @param sl scan lines.
 ** @return number of scan lines.
 **/
/*int CalScanLine(int nsi, int msi, float si, float sr, float *lad, int nvd, Camera *cam,
                int msr, LaneStateModel *lsm, ScanLine *sl)
{
	int n = nsi;
	
	// Calculate x-position of the nearest two scan lines.
	float dist = lad[0];
	double temp = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
	double yll = -lsm->b / 2 + temp;
	double ylr = lsm->b / 2 + temp;
	temp = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
	int xbl = (int)(cam->fx * yll / dist + temp + 0.5f);
	int xbr = (int)(cam->fx * ylr / dist + temp + 0.5f);
	
	if (xbl < 0 || xbr > cam->xreso - 1) {
		fprintf(stderr, "X position of the first scan line<%d,%d> is out of range!\n", xbl, xbr);
		// return 0;
	}
	
	int nsl = 0;
	sl[nsl].x = xbl;
	sl[nsl].y = cam->yreso - 1;
	sl[nsl].radius = msr;
	sl[nsl].bx = -1;
	sl[nsl].px = -1;
	sl[nsl].dir = CaLaneDirection(cam, lsm, dist, -1);
	sl[nsl + n + 1].x = xbr;
	sl[nsl + n + 1].y = sl[nsl].y;
	sl[nsl + n + 1].radius = msr;
	sl[nsl + n + 1].bx = -1;
	sl[nsl + n + 1].px = -1;
	sl[nsl + n + 1].dir = CaLaneDirection(cam, lsm, dist, 1);
	nsl++;
	
	// Scale factor (p) of scan line radius.
	float p = powf(sr, 1.0f / n);	// p=b(i+1)/b(i)=(b(n)/b(1))^(1/n)
	
	for (int i = 1; i <= n; i++) {
		// Y-position of scan lines.
		int interval = (int)(msi * (1 - pow(si, i)) / (1 - si) + 0.5f);
		int y = cam->yreso - interval;
		
		// Calculate x-position of further scan lines.
		float dist = lad[cam->yreso - 1 - y];
		double temp = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
		double yll = -lsm->b / 2 + temp;
		double ylr = lsm->b / 2 + temp;
		temp = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
		int xbl = (int)(cam->fx * yll / dist + temp + 0.5f);
		int xbr = (int)(cam->fx * ylr / dist + temp + 0.5f);
		
		// Calculate scan line radius.
		int radius = (int)(msr * pow(p, i));
		
		if (xbl < 0 || xbr > cam->xreso - 1) {
			fprintf(stderr, "X position of scan line<%d,%d,%d> is out of range!\n", xbl, xbr, y);
			// break;
		}
		
		sl[nsl].x = xbl;
		sl[nsl].y = y;
		sl[nsl].radius = radius;
		sl[nsl].bx = -1;
		sl[nsl].px = -1;
		sl[nsl].dir = CaLaneDirection(cam, lsm, dist, -1);
		sl[nsl + n + 1].x = xbr;
		sl[nsl + n + 1].y = y;
		sl[nsl + n + 1].radius = radius;
		sl[nsl + n + 1].bx = -1;
		sl[nsl + n + 1].px = -1;
		sl[nsl + n + 1].dir = CaLaneDirection(cam, lsm, dist, 1);
		nsl++;
	}
		
	return (nsl << 1);
}*/
int CalScanLine(int nsi, int msi, float si, float sr, float *lad, int nvd, Camera *cam,
                int msr, LaneStateModel *lsm, int side, int yfs, ScanLine *sl)
{
	int n = nsi;
	if (n <= 0) {
		return 0;
	}
	
	// Calculate x-position of the first scan line.
	float dist = lad[cam->yreso - 1 - yfs];
	double temp = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
	double yl = side * lsm->b / 2 + temp;
	temp = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
	int xb = (int)(cam->fx * yl / dist + temp + 0.5f);
	
	int nsl = 0;
	sl[nsl].x = xb;
	sl[nsl].y = yfs;
	sl[nsl].radius = msr;
	sl[nsl].bx = -1;
	sl[nsl].px = -1;
	sl[nsl].dir = CaLaneDirection(cam, lsm, dist, side);
	nsl++;
	
	// Scale factor (p) of scan line radius.
	float p = powf(sr, 1.0f / n);	// p=b(i+1)/b(i)=(b(n)/b(1))^(1/n)
	
	for (int i = 1; i <= n; i++) {
		// Y-position of scan lines.
		int interval = (int)(msi * (1 - pow(si, i)) / (1 - si) + 0.5f);
		int y = yfs - interval;
		
		// Calculate x-position of further scan lines.
		float dist = lad[cam->yreso - 1 - y];
		double temp = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
		double yl = side * lsm->b / 2 + temp;
		temp = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
		int xb = (int)(cam->fx * yl / dist + temp + 0.5f);
		
		// Calculate scan line radius.
		int radius = (int)(msr * pow(p, i));
		
		sl[nsl].x = xb;
		sl[nsl].y = y;
		sl[nsl].radius = radius;
		sl[nsl].bx = -1;
		sl[nsl].px = -1;
		sl[nsl].dir = CaLaneDirection(cam, lsm, dist, side);
		nsl++;
	}
	
	// Limit scan line radius.
	// dist = lad[cam->yreso - 1 - sl[nsl - 1].y];
	// temp = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
	// double yl0 =  side * lsm->b / 2 + temp;
	// double yl1 = -side * lsm->b / 2 + temp;
	// temp = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
	// int xb0 = (int)(cam->fx * yl0 / dist + temp + 0.5f);
	// int xb1 = (int)(cam->fx * yl1 / dist + temp + 0.5f);
	// float minrad = abs(xb0 - xb1) / 2.0f;
	// sr = minrad / msr;
	// p = powf(sr, 1.0f / n);
	// 
	// for (int i = 1; i <= n; i++) {
	// 	int radius = (int)(msr * pow(p, i));
	// 	sl[i].radius = radius;
	// }
	
	return nsl;
}

/** @brief Calculate tangent direction of projected lane boundary.
 **        The direction range is (-pi,pi].
 ** @param cam camera model.
 ** @param lsm lane state model.
 ** @param dist look ahead distance.
 ** @param side left(side=-1) or right(side=1) side.
 ** @return the tangent direction of projected lane boundary.
 **/
float CaLaneDirection(Camera *cam, LaneStateModel *lsm, float dist, int side)
{
	// Calculate dy/dl.
	double dyl = -cam->fy * cam->h / (dist * dist) - cam->fy * lsm->cv0 / 2;
	
	// Calculate dx/dl.
	double dxl = -cam->fx * (side * lsm->b / 2 - lsm->y0) / (dist * dist) + \
		cam->fx * lsm->ch0 / 2 + cam->fx * lsm->ch1 * dist / 3;

	return (float)atan2(dyl, dxl);	
}

/** @brief Initialize Jacobian matrix of measurement function.
 ** @param cam camera model.
 ** @param lad look ahead distance.
 ** @param sl scan lines.
 ** @param nsl number of scan lines.
 ** @param side left(=-1) or right(=1)side.
 ** @param lsm state prior.
 ** @param jm Jacobian matrix.
 ** @param jmw width of Jacobian matrix.
 ** @param jmh height of Jacobian matrix.
 **/
/*void CalMeasureJacoMatrix(Camera *cam, float *lad, ScanLine *sl, int nsl,
                          LaneStateModel *lsm, double *jm, int jmw, int jmh)
{
	int hjmh = jmh >> 1;
	
	// Left track's scan lines.
	for (int i = 0; i < hjmh; i++) {
		float dist = lad[cam->yreso - 1 - sl[i].y];
		double *ptr = jm + i * jmw;
		*(ptr++) = cam->fx;
		double dxl = cam->fx * (lsm->b/2 + lsm->y0) / dist / dist + cam->fx * lsm->ch0 / 2 +
			cam->fx * lsm->ch1 * dist;
		double dphi2l = cam->h / dist / dist + lsm->cv0 / 2;
		*(ptr++) = dxl / dphi2l;
		*(ptr++) = -cam->fx / dist;
		*(ptr++) = -cam->fx / (2 * dist);
		*(ptr++) = cam->fx * dist / 2;
		*(ptr++) = cam->fx * dist * dist / 6;
		double dcv0l = 2 * ((sl[i].y - cam->cy) / cam->fy - lsm->phi2) / dist / dist - 4 * cam->h / dist / dist / dist;
		*(ptr)   = dxl / dcv0l;
	}
	
	// Right track's scan lines.
	for (int i = hjmh; i < jmh; i++) {
		float dist = lad[cam->yreso - 1 - sl[i].y];
		double *ptr = jm + i * jmw;
		*(ptr++) = cam->fx;
		double dxl = cam->fx * (-lsm->b/2 + lsm->y0) / dist / dist + cam->fx * lsm->ch0 / 2 +
			cam->fx * lsm->ch1 * dist;
		double dphi2l = cam->h / dist / dist + lsm->cv0 / 2;
		*(ptr++) = dxl / dphi2l;
		*(ptr++) = -cam->fx / dist;
		*(ptr++) = cam->fx / (2 * dist);
		*(ptr++) = cam->fx * dist / 2;
		*(ptr++) = cam->fx * dist * dist / 6;
		double dcv0l = 2 * ((sl[i].y - cam->cy) / cam->fy - lsm->phi2) / dist / dist - 4 * cam->h / dist / dist / dist;
		*(ptr)   = dxl / dcv0l;
	}
}*/
void CalMeasureJacoMatrix(Camera *cam, float *lad, ScanLine *sl, int nsl, int side,
                          LaneStateModel *lsm, double *jm, int jmw, int jmh)
{	
	for (int i = 0; i < jmh; i++) {
		float dist = lad[cam->yreso - 1 - sl[i].y];
		double *ptr = jm + i * jmw;
		*(ptr++) = cam->fx;
		double dxl = cam->fx * (-side * lsm->b/2 + lsm->y0) / dist / dist + cam->fx * lsm->ch0 / 2 +
			cam->fx * lsm->ch1 * dist;
		double dphi2l = cam->h / dist / dist + lsm->cv0 / 2;
		*(ptr++) = dxl / dphi2l;
		*(ptr++) = -cam->fx / dist;
		*(ptr++) = side * cam->fx / (2 * dist);
		*(ptr++) = cam->fx * dist / 2;
		*(ptr++) = cam->fx * dist * dist / 6;
		double dcv0l = 2 * ((sl[i].y - cam->cy) / cam->fy - lsm->phi2) / dist / dist - 4 * cam->h / dist / dist / dist;
		*(ptr)   = dxl / dcv0l;
	}
}

/** @brief Railway tracking thread.
 ** @param param thread parameter.
 **/
void *RailwayTrackThread(void *param)
{
	RailwayTracker *self = (RailwayTracker *)param;
	int ruimgsz = RoundupPowerOf2(self->camera.yreso * self->camera.xreso * sizeof(unsigned short));
	int rutklsz = RoundupPowerOf2(sizeof(List));
	
	cv::KalmanFilter kalman(self->dynadim, self->measdim, 0, CV_64F);
	
	// Initialize state transition matrix (=Jacobian matrix of transition function).
	kalman.transitionMatrix =
		(cv::Mat_<double>(self->dynadim, self->dynadim) <<
		1, 0, 0, 0, 0, 0, 0,		// phi1(t)=phi1(t-1)
		0, 1, 0, 0, 0, 0, 0,		// phi2(t)=phi2(t-1)
		0, 0, 1, 0, 0, 0, 0,		// y0(t)=y0(t-1)
		0, 0, 0, 1, 0, 0, 0,		// b(t)=b(t-1)
		0, 0, 0, 0, 1, 1, 0,		// ch0(t)=ch0(t-1)+ch1(t-1)
		0, 0, 0, 0, 0, 1, 0,		// ch1(t)=ch1(t-1)
		0, 0, 0, 0, 0, 0, 1);		// cv0(t)=cv0(t-1)
	
	// Initialize Jacobian matrix of measurement function.
	CalMeasureJacoMatrix(&self->camera, self->ladist, self->scanline,              self->nlsl, -1,
		&self->s0, (double *)kalman.measurementMatrix.data,                              self->dynadim, self->nlsl);
	CalMeasureJacoMatrix(&self->camera, self->ladist, self->scanline + self->nlsl, self->nrsl,  1,
		&self->s0, (double *)kalman.measurementMatrix.data + self->nlsl * self->dynadim, self->dynadim, self->nrsl);
	
	// Initialize process noise covariance matrix. (Q)
	InitProcessNoiseCov("RailwayMonitor.json", (double *)kalman.processNoiseCov.data,
		kalman.processNoiseCov.rows * kalman.processNoiseCov.cols);
	std::cout << "processNoiseCov=" << std::endl << kalman.processNoiseCov << std::endl;
	
	// Initialize measurement noise covariance matrix. (R)
	InitMeasurementNoiseCov("RailwayMonitor.json", (double *)kalman.measurementNoiseCov.data,
		kalman.measurementNoiseCov.rows * kalman.measurementNoiseCov.cols);
	std::cout << "measurementNoiseCov=diag(" << kalman.measurementNoiseCov.ptr<double>(0)[0] << ")" << std::endl;
	
	// Initialize posteriori error estimate covariance matrix.	(P)
	InitErrorCovPost("RailwayMonitor.json", (double *)kalman.errorCovPost.data,
		kalman.errorCovPost.rows * kalman.errorCovPost.cols);
	std::cout << "errorCovPost=" << std::endl << kalman.errorCovPost << std::endl;
	
	// Initialize posteriori (corrected) state.
	kalman.statePost = (cv::Mat_<double>(self->dynadim, 1) <<
		self->s0.phi1, self->s0.phi2, self->s0.y0, self->s0.b, self->s0.ch0, self->s0.ch1, self->s0.cv0);
	
	LaneStateModel statepost;		// State posteriori
	LaneStateModel statepre;		// State prior
	
	// 1.Save state posteriori for step 2 and 4.
	State2Model((double *)kalman.statePost.data, &statepost);
	
	List *tracklist = (List *)self->tracklist;
	List *tracklist1 = (List *)self->tracklist1;
	List *tracklist2 = (List *)self->tracklist2;
	cv::Mat measurement(self->measdim, 1, CV_64FC1, self->measurement);
	
	int left_cutoff = 0;
	int right_cutoff = 0;
	
	while (self->runswitch) {
		// Read raw image from ring buffer.
		int red = fifo_get(self->rawring, (char *)self->rawimg, ruimgsz);
		if (red != ruimgsz) {
			Sleep(1);
			continue;
		}
		
		ListInit(tracklist);
		ListInit(tracklist1);
		ListInit(tracklist2);
		int reinitialize = 0;
		int nsl = self->measdim;
		int numleftscaninter;
		int numrightscaninter;
		float maxleftscaninter;
		float maxrightscaninter;
		float leftinterscale;
		float rightinterscale;
		int nlsl = 0, nrsl = 0;
		
		if (self->disable) {
			goto eot;
		}
		
		// 2.Update Jacobian matrix of transition function.
		// The Jacobian matrix is the same as transition matrix, so do nothing.
		
		// 3.Make prediction.
		kalman.predict();
		
		// 4.Recalculate state prediction using the transition function.
		// The Jacobian matrix is the same as transition matrix, so do nothing.
		
		// Blur image width gaussian kernel.
		Blur(self->rawimg, self->camera.xreso, self->camera.yreso, self->blurimg);
		
		// Calculate gradient magnitude and direction.
		CalGradient(self->blurimg, self->camera.xreso, self->camera.yreso,
			self->magximg, self->magyimg, self->magimg, self->dirimg);
		
		// Calculate look ahead distance corresponding to the state model.
		State2Model((double *)kalman.statePre.data, &statepre);
		int nvd = CaLoodaheaDist(&self->camera, &statepre, self->ladist, self->camera.yreso);
		int ylfs, nlvd, yrfs, nrvd;
		ChooseValidLookahead(&self->camera, &statepre, self->ladist, nvd, &ylfs, &nlvd, &yrfs, &nrvd);
		// printf("ylfs=%d,nlvd=%d,yrfs=%d,nrvd=%d.\n", ylfs, nlvd, yrfs, nrvd);
				
		nlvd = (nlvd < ylfs - left_cutoff) ? nlvd : ylfs - left_cutoff;
		nrvd = (nrvd < yrfs - right_cutoff) ? nrvd : yrfs - right_cutoff;
		
		// Calculate scan line parameters.
		int minsumscaninter = self->numscaninter * self->minscaninter * 3;
		if (nlvd > minsumscaninter && nrvd > minsumscaninter) {
			// Check state model validation.
			if (statepre.y0 < self->lateraloffsetlowlim || statepre.y0 > self->lateraloffsethiglim) {
				fprintf(stderr, "Lateral offset error!\n");
				reinitialize = 1;
				goto eot;
			}
			
			// Check the first scan line position's validation.
			if (ylfs < 2 * self->camera.yreso / 3 || yrfs < 2 * self->camera.yreso / 3) {
				fprintf(stderr, "The first scan line's position is impossible!\n");
				reinitialize = 1;
				goto eot;
			}
			
			numleftscaninter = self->numscaninter;
			numrightscaninter = self->numscaninter;
			// Left and right first scan lines are aligned.
			if (ylfs == yrfs) {
				nvd = (nlvd < nrvd) ? nlvd : nrvd;
				float sumscaninter = nvd * self->rslad;
				sumscaninter = (sumscaninter < self->maxsumscaninter) ? sumscaninter : self->maxsumscaninter;
				CalScanLineParam((float)numleftscaninter, (float)self->minscaninter, sumscaninter,
					&maxleftscaninter, &leftinterscale);
				maxrightscaninter = maxleftscaninter;
				rightinterscale = leftinterscale;
			} else { // Left and right first scan lines are not aligned.
				float sumscaninter = nlvd * self->rslad;
				sumscaninter = (sumscaninter < self->maxsumscaninter) ? sumscaninter : self->maxsumscaninter;
				CalScanLineParam((float)numleftscaninter, (float)self->minscaninter, sumscaninter,
					&maxleftscaninter, &leftinterscale);
				sumscaninter = nrvd * self->rslad;
				sumscaninter = (sumscaninter < self->maxsumscaninter) ? sumscaninter : self->maxsumscaninter;
				CalScanLineParam((float)numrightscaninter, (float)self->minscaninter, sumscaninter,
					&maxrightscaninter, &rightinterscale);
			}
		} else if (nlvd > minsumscaninter) {
			fprintf(stderr, "Track left track only!\n");
			numleftscaninter = (self->numscaninter << 1) + 1;
			numrightscaninter = 0;
			float sumscaninter = (float)nlvd;
			sumscaninter = (sumscaninter < self->maxsumscaninter) ? sumscaninter : self->maxsumscaninter;
			CalScanLineParam((float)numleftscaninter, (float)self->minscaninter, sumscaninter,
				&maxleftscaninter, &leftinterscale);
			maxrightscaninter = 0;
			rightinterscale = 1;
		} else if (nrvd > minsumscaninter) {
			fprintf(stderr, "Track right track only!\n");
			numleftscaninter = 0;
			numrightscaninter = (self->numscaninter << 1) + 1;
			float sumscaninter = (float)nrvd;
			sumscaninter = (sumscaninter < self->maxsumscaninter) ? sumscaninter : self->maxsumscaninter;
			CalScanLineParam((float)numrightscaninter, (float)self->minscaninter, sumscaninter,
				&maxrightscaninter, &rightinterscale);
			maxleftscaninter = 0;
			leftinterscale = 1;
		} else {
			fprintf(stderr, "Track information insufficiently!\n");
			reinitialize = 1;
			goto eot;
		}
		
		// Calculate scan lines corresponding to the state model.
		nlsl = CalScanLine(numleftscaninter, (int)maxleftscaninter, leftinterscale, self->scaler, self->ladist, 
			nlvd, &self->camera, self->maxscanrad, &statepre, -1, ylfs, self->scanline);
		nrsl = CalScanLine(numrightscaninter, (int)maxrightscaninter, rightinterscale, self->scaler, self->ladist,
			nrvd, &self->camera, self->maxscanrad, &statepre,  1, yrfs, self->scanline + nlsl);

		// Back to straight track, make sure that no track missing.
		if ((nlsl == 0 && self->scanline[0].x > self->rightstraightrackfloor) ||
			(nrsl == 0 && self->scanline[0].x < self->leftstraightrackceil)) {
			fprintf(stderr, "Miss tracking one of the track!\n");
			reinitialize = 1;
			goto eot;
		}
			
		// Extract track border on scan lines.
		ExtractTrackBorder(self->magimg, self->dirimg, self->camera.xreso, self->camera.yreso,
			self->scanline, nsl);
		
		// Check goodness of fit of the state model.
		if (CheckGoodnessOfFit(self->scanline, nsl, self->goodfithresh)) {
			fprintf(stderr, "State model fit bad!\n");
			reinitialize = 1;
			goto eot;
		}
		
		// 5.Calculate Jacobian matrix of measurement function.
		CalMeasureJacoMatrix(&self->camera, self->ladist, self->scanline,        nlsl, -1, &statepre,
			(double *)kalman.measurementMatrix.data,                        self->dynadim, nlsl);
		CalMeasureJacoMatrix(&self->camera, self->ladist, self->scanline + nlsl, nrsl,  1, &statepre,
			(double *)kalman.measurementMatrix.data + nlsl * self->dynadim, self->dynadim, nrsl);
					
		// 6.Make correction.
		GetMeasurement(self->scanline, nsl, (double *)measurement.data);
		kalman.correct(measurement);
						
		// 7.Recalculate residual error.
		CalResidualError(&self->camera, self->ladist, self->scanline,        nlsl, -1, (double *)kalman.statePre.data,
			(double *)kalman.temp5.data,                            kalman.temp5.cols, nlsl);
		CalResidualError(&self->camera, self->ladist, self->scanline + nlsl, nrsl,  1, (double *)kalman.statePre.data,
			(double *)kalman.temp5.data + nlsl * kalman.temp5.cols, kalman.temp5.cols, nrsl);
		
		// 8.Recalculate state posteriori.		
		kalman.statePost = kalman.statePre + kalman.gain * kalman.temp5;
		
		// 1.Save state posteriori for step 2 and 4.
		State2Model((double *)kalman.statePost.data, &statepost);
		
		// Calculate track boundary corresponding to current state model.
		CalTrackBoundary(self->magimg, self->dirimg, &self->camera, self->scanline,        nlsl, -1,
			self->pedwarnwidth, self->obswarnwidth, &statepost, tracklist);
		CalTrackBoundary(self->magimg, self->dirimg, &self->camera, self->scanline + nlsl, nrsl,  1,
			self->pedwarnwidth, self->obswarnwidth, &statepost, tracklist);

		eot:
		// Write track list to ring buffer of obstacle detector.
		ListCopy(tracklist, tracklist1, sizeof(struct Track));
		if (ObstacleDetectorPutTrack(self->odetector, (char *)tracklist1, rutklsz)) {
			fprintf(stderr, "ObstacleDetectorPutTrack fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		// Write track list to ring buffer of pedestrian detector.
		ListCopy(tracklist, tracklist2, sizeof(struct Track));
		if (PedestrianDetectorPutTrack(self->pdetector, (char *)tracklist2, rutklsz)) {
			fprintf(stderr, "PedestrianDetectorPutTrack fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		// Write track list to ring buffer of self.
		int wrt = fifo_put(self->trackring, (const char *)tracklist, rutklsz);
		if (wrt != rutklsz) {
			fprintf(stderr, "fifo_put fail[%s:%d].\n", __FILE__, __LINE__);
		}
		
		if (!self->disable) {
#ifdef _DEBUG
			ProjectStatePost(&self->camera, &statepost, self->scanline,        nlsl, -1);
			ProjectStatePost(&self->camera, &statepost, self->scanline + nlsl, nrsl,  1);
			DrawTrackerScanLines(self, (unsigned char *)self->rawimg);
#endif
			if (reinitialize) {
				InitExtendedKalmanFilter(self, (double *)kalman.statePost.data, (double *)kalman.errorCovPost.data,
					(double *)kalman.measurementMatrix.data, self->dynadim, self->measdim);
				statepost = self->s0;
				left_cutoff = 0; right_cutoff = 0;
				continue;
			}
			
			// Update cutoff lines.
			left_cutoff = UpdateCutoffLine(self->scanline, nlsl, self->directionbiasthresh);
			right_cutoff = UpdateCutoffLine(self->scanline + nlsl, nrsl, self->directionbiasthresh);
			
			// Save the last Extended Kalman Filtr run parameters.
			SaveLaneState("LaneState.txt", &statepost);
			SaveErrorCovPost("ErrorCovPost.txt", (double *)kalman.errorCovPost.data, kalman.errorCovPost.cols,
				kalman.errorCovPost.rows);
		}
	}

	return 0;
}

/** @brief Blur image with gaussian kernel.
 **        The kernel size is 3x3.
 ** @param src source single channel image.
 ** @param width image width.
 ** @param height image height.
 ** @param dst destination blurred image.
 **/
void Blur(unsigned short *src, int width, int height, unsigned short *dst)
{
	cv::Mat _src(height, width, CV_16UC1, src);
	cv::Mat _dst(height, width, CV_16UC1, dst);
	
	cv::GaussianBlur(_src, _dst, cv::Size(3, 3), 0, 0);
}

/** @brief Calculate gradient of image.
 ** @param src source single channel image.
 ** @param width image width.
 ** @param height image height.
 ** @param magx x-gradient magnitude.
 ** @param magy y-gradient magnitude.
 ** @param mag gradient magnitude.
 ** @param dir gradient direction, direction range is [0,2*pi].
 **/
void CalGradient(unsigned short *src, int width, int height, float *magx,
                 float *magy, float *mag, float *dir)
{
	cv::Mat _src(height, width, CV_16UC1, src);
	cv::Mat _magx(height, width, CV_32FC1, magx);
	cv::Mat _magy(height, width, CV_32FC1, magy);
	cv::Mat _mag(height, width, CV_32FC1, mag);
	cv::Mat _dir(height, width, CV_32FC1, dir);
	
	cv::Sobel(_src, _magx, CV_32FC1, 1, 0, 3);
	cv::Sobel(_src, _magy, CV_32FC1, 0, 1, 3);
	
	cv::cartToPolar(_magx, _magy, _mag, _dir);
}

/** @brief Extract track border.
 ** @param mag gradient magnitude.
 ** @param dir gradient direction.
 ** @param width image width.
 ** @param height image height.
 ** @param sl scan lines.
 ** @param nsl number of scan lines.
 **/
void ExtractTrackBorder(float *mag, float *dir, int width, int height,
                        ScanLine *sl, int nsl)
{
	float pi = 3.14159265358979323f;
	// int hnsl = nsl >> 1;
		
	for (int i = 0; i < nsl; i++) {
		// Normalize boundary direction in [-pi/2,pi/2].
		float pbtd = sl[i].dir;
		if (pbtd < -pi / 2) {
			pbtd += pi;
		} else if (pbtd > pi / 2) {
			pbtd -= pi;
		}
		
		int bestx = -1;
		float bested;
		float bestscore = -99999;
		// int middlex = (sl[i].x + sl[(i + hnsl) % nsl].x) >> 1;
		for (int x = sl[i].x - sl[i].radius; x <= sl[i].x + sl[i].radius; x++) {
			if (x < 0 || x > width - 1/* || (i < hnsl && x > middlex) || (i >= hnsl && x < middlex)*/) {
				continue;
			}
			
			// Calculate edge direction at this point.
			float ed = 0;
			float gd = *(dir + sl[i].y * width + x);
			if (gd < pi) {
				ed = gd - pi / 2;
			} else if (gd > pi) {
				ed = gd - 3 * pi / 2;
			}
			
			// Calculate matching score with predicted lane boundary.
			float sigma = sl[i].radius * 2.0f / 3;		// Pauta criterion
			float score = *(mag + sl[i].y * width + x) * pow(fabs(cos(pbtd - ed)), 0.2) * GaussWeight(sigma, x - sl[i].x);
			// if (i != 0 && i != hnsl) {
			// if ((i < hnsl && i > 8) || (i >= hnsl && i > hnsl + 8)) {
				// score *= MatchPredictDirection(&sl[i - 8], &sl[i], x);
			// }
			
			if (score > bestscore) {
				bestscore = score;
				bestx = x;
				bested = ed;
			}
		}

		// if (sl[i].x >= 0 && sl[i].x <= width - 1) {
		if (bestx >= 0 && bestx <= width - 1) {
			sl[i].bx = bestx;
			sl[i].dir = pbtd;
			sl[i].edir = bested;
		}/* else {
			printf("Not detected measurement!\n");
			sl[i].bx = sl[i].x;
		}*/
	}
}

/** @brief Calculate gaussian weight.
 ** @param sigma standard deviation of normal gaussian kernel.
 ** @param dist distance to mean.
 **/
float GaussWeight(float sigma, int dist)
{
	return exp(-dist*dist/sigma/sigma);
}

/** @brief Calculate direction similarity between prediction and measurement.
 ** @param prev previous scan line.
 ** @param curr current scan line.
 ** @param x x of candidate on current scan line.
 ** @return direction similarity.
 **/
float MatchPredictDirection(ScanLine *prev, ScanLine *curr, int x)
{
	float pdir = atan2f((float)curr->y - prev->y, (float)curr->x - prev->x);
	float mdir = atan2f((float)curr->y - prev->y, (float)x - prev->bx);
	float simi = fabs(cos(mdir - pdir));
	
	return simi; // > 0.866 ? simi : 0;
}

/** @brief Check goodness of state model fit.
 ** @param sl scan line.
 ** @param nsl number of scan line.
 ** @param thresh goodness threshold.
 ** @return  0 if good,
 **         -1 if bad.
 **/
int CheckGoodnessOfFit(ScanLine *sl, int nsl, float thresh)
{
	float errsum = 0;
	for (int i = 0; i < nsl; i++) {
		errsum += abs(sl[i].bx - sl[i].x); // * (1 - pow(fabs(cos(sl[i].dir - sl[i].edir)), 2));
	}
	
	float errmean = errsum / nsl;
	if (errmean < thresh) {
		return 0;
	} else {
		printf("errmean=%f.\n", errmean);
		return -1;
	}
}

/** @brief Get measurement from scan lines.
 ** @param sl scan lines.
 ** @param nsl number of scan lines.
 ** @param measure measurement.
 **/
void GetMeasurement(ScanLine *sl, int nsl, double *measure)
{
	for (int i = 0; i < nsl; i++) {
		measure[i] = (double)sl[i].bx;
	}
}

/** @brief Calculate residual error between measurement and prediction.
 ** @param cam camera model.
 ** @param lad look ahead distance.
 ** @param sl scan lines.
 ** @param nsl number of scan lines.
 ** @param side left(=-1) or right(=1)side.
 ** @param statepre state prediction.
 ** @param jmm Jacobian matrix of measurement function.
 ** @param jmw width of Jacobian matrix.
 ** @param jmh height of Jacobian matrix.
 **/ 
/*void CalResidualError(Camera *cam, float *lad, ScanLine *sl, int nsl,
                      double *statepre, double *jmm, int jmw, int jmh)
{
	int hnsl = nsl >> 1;
	
	// Left track.
	for (int i = 0; i < hnsl; i++) {		
		// yl=-b/2+ch0*dist^2/2+ch1*dist^3/6.
		float dist = lad[cam->yreso - 1 - sl[i].y];
		double yl = -statepre[3] / 2 + statepre[4] * dist * dist / 2 +
			statepre[5] * dist * dist * dist / 6;
		
		// Calculate predicted x on the scan line.
		double predictx = cam->fx * (yl - statepre[2] + statepre[0] * dist) / dist + cam->cx;

		jmm[i] = sl[i].bx - predictx;
	}
	
	// Right track.
	for (int i = hnsl; i < nsl; i++) {
		// yl=b/2+ch0*dist^2/2+ch1*dist^3/6.
		float dist = lad[cam->yreso - 1 - sl[i].y];
		double yl = statepre[3] / 2 + statepre[4] * dist * dist / 2 +
			statepre[5] * dist * dist * dist / 6;
		
		// Calculate predicted x on the scan line.
		double predictx = cam->fx * (yl - statepre[2] + statepre[0] * dist) / dist + cam->cx;

		jmm[i] = sl[i].bx - predictx;
	}
}*/
void CalResidualError(Camera *cam, float *lad, ScanLine *sl, int nsl, int side,
                      double *statepre, double *jmm, int jmw, int jmh)
{
	for (int i = 0; i < nsl; i++) {		
		// yl=-b/2+ch0*dist^2/2+ch1*dist^3/6.
		float dist = lad[cam->yreso - 1 - sl[i].y];
		double yl = side * statepre[3] / 2 + statepre[4] * dist * dist / 2 +
			statepre[5] * dist * dist * dist / 6;
		
		// Calculate predicted x on the scan line.
		double predictx = cam->fx * (yl - statepre[2] + statepre[0] * dist) / dist + cam->cx;

		jmm[i] = sl[i].bx - predictx;
	}
}

/** @brief Transform state vector to state model.
 ** @param state state vector.
 ** @param lsm state model.
 **/
void State2Model(double *state, LaneStateModel *lsm)
{
	lsm->phi1 = state[0];
	lsm->phi2 = state[1];
	lsm->y0   = state[2];
	lsm->b    = state[3];
	lsm->ch0  = state[4];
	lsm->ch1  = state[5];
	lsm->cv0  = state[6];
}

/** @brief Solve quadratic equation.
 ** @param p1 squared item.
 ** @param p2 one time item.
 ** @param p3 constant item.
 ** @param sol solution of the equation.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int SolveQuadraticEquation(double p1, double p2, double p3, double *sol)
{
	double _sol;
	if (fabs(p1) > 1e-10) {
		double delta = p2 * p2 - 4 * p1 * p3;
		if (delta > 0) {
			_sol = (-p2 - sqrt(delta)) / (2 * p1);
		} else {
			return -1;
		}
	} else {
		if (fabs(p2) > 1e-30) {
			_sol = -p3 / p2;
		} else {
			return -1;
		}
	}
	
	*sol = _sol;
	
	return 0;
}

/** @brief Project state posteriori on scan lines.
 ** @param cam camera model.
 ** @param lsm lane state model.
 ** @param sl scan lines.
 ** @param nsl number of scan lines.
 ** @param side left(=-1) or right(=1)side.
 **/
/*void ProjectStatePost(Camera *cam, LaneStateModel *lsm, ScanLine *sl, int nsl)
{
	int hnsl = nsl >> 1;
	
	for (int i = 0; i < hnsl; i++) {
		double p1 = -cam->fy * lsm->cv0 / 2;
		double p3 = cam->fy * cam->h;
		double p2 = cam->fy * lsm->phi2 + cam->cy - sl[i].y;
		
		double dist = -1;
		if (SolveQuadraticEquation(p1, p2, p3, &dist) || dist < 0) {
			continue;
		}
		
		double temp = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
		double yll = -lsm->b / 2 + temp;
		double ylr = lsm->b / 2 + temp;
		temp = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
		int xbl = (int)(cam->fx * yll / dist + temp + 0.5f);
		int xbr = (int)(cam->fx * ylr / dist + temp + 0.5f);
		
		if (xbl >= 0) {
			sl[i].px = xbl;
		}
		
		if (xbr < cam->xreso) {
			sl[i + hnsl].px = xbr;
		}
	}
}*/
void ProjectStatePost(Camera *cam, LaneStateModel *lsm, ScanLine *sl, int nsl, int side)
{	
	for (int i = 0; i < nsl; i++) {
		double p1 = -cam->fy * lsm->cv0 / 2;
		double p3 = cam->fy * cam->h;
		double p2 = cam->fy * lsm->phi2 + cam->cy - sl[i].y;
		
		double dist = -1;
		if (SolveQuadraticEquation(p1, p2, p3, &dist) || dist < 0) {
			continue;
		}
		
		double temp = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
		double yl = side * lsm->b / 2 + temp;
		temp = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
		int xb = (int)(cam->fx * yl / dist + temp + 0.5f);
		
		if (xb >= 0 && xb < cam->xreso) {
			sl[i].px = xb;
		}
	}
}

/** @brief Save last lane state as local file.
 ** @param path lane state file path.
 ** @param lsm lane state model.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int SaveLaneState(const char *path, LaneStateModel *lsm)
{
	FILE *fp = fopen(path, "w");
	if (!fp) {
		fprintf(stderr, "fopen fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	fprintf(fp, "%e\n", lsm->phi1);
	fprintf(fp, "%e\n", lsm->phi2);
	fprintf(fp, "%e\n", lsm->y0);
	fprintf(fp, "%e\n", lsm->b);
	fprintf(fp, "%e\n", lsm->ch0);
	fprintf(fp, "%e\n", lsm->ch1);
	fprintf(fp, "%e\n", lsm->cv0);
	
	fclose(fp);
	
	return 0;
}

/** @brief Save error covariance posteriori as local file.
 ** @param path error covariance posteriori file path.
 ** @param ecp error covariance posteriori.
 ** @param width width of error covariance posteriori matrix.
 ** @param height height of error covariance posteriori matrix.
 ** @return  0 if success,
 **         -1 if fail.
 **/
int SaveErrorCovPost(const char *path, double *ecp, int width, int height)
{
	FILE *fp = fopen(path, "w");
	if (!fp) {
		fprintf(stderr, "fopen fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			fprintf(fp, "%e ", ecp[y * width + x]);
		}
		fputs("\n", fp);
	}
	
	fclose(fp);
	
	return 0;
}

/** @brief Calculate track boundary.
 ** @param mag gradient magnitude image.
 ** @param dir gradient direction image.
 ** @param cam camera model.
 ** @param sl scan lines.
 ** @param nsl number of scan lines.
 ** @param side left(=-1) or right(=1)side.
 ** @param pww pedestrian warning region width.
 ** @param oww obstacle warning region width.
 ** @param lsm lane state model.
 ** @param track track boundary list.
 **/ 
/*void CalTrackBoundary(float *mag, float *dir, Camera *cam, ScanLine *sl, int nsl,
                      float pww, float oww, LaneStateModel *lsm, List *tracklist)
{	
	int hnsl = nsl >> 1;
	
	// Left track boundary.	
	float k = (sl[hnsl - 1].radius - sl[0].radius) / ((float)sl[hnsl - 1].y - sl[0].y);
	for (int yb = sl[0].y; yb >= sl[hnsl - 1].y; yb--) {
		double p1 = -cam->fy * lsm->cv0 / 2;
		double p3 = cam->fy * cam->h;
		double p2 = cam->fy * lsm->phi2 + cam->cy - yb;
		
		double dist = -1;
		if (SolveQuadraticEquation(p1, p2, p3, &dist) || dist < 0) {
			continue;
		}
		
		double temp1 = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
		double yl = -lsm->b / 2 + temp1;
		double temp2 = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
		int xb = (int)(cam->fx * yl / dist + temp2 + 0.5f);
		
		if (xb < 0 || xb > cam->xreso - 1) {
			continue;
		}
		
		int radius = (int)((yb - sl[0].y) * k + sl[0].radius);
		float direction = CaLaneDirection(cam, lsm, (float)dist, -1);
		ScanLine ssl = {xb, yb, radius, 0, 0, direction};
		ExtractTrackBorder(mag, dir, cam->xreso, cam->yreso, &ssl, 1);

		double yw = -pww / 2 + temp1;
		int xpw = (int)(cam->fx * yw / dist + temp2 + 0.5f);
		
		yw = -oww / 2 + temp1;
		int xow = (int)(cam->fx * yw / dist + temp2 + 0.5f);

		struct Track track{xb, yb, ssl.bx, xpw, xow, 0, (float)dist};
		TrackListAddTail(tracklist, &track);
	}

	// Right track boundary.	
	k = (sl[nsl - 1].radius - sl[hnsl].radius) / ((float)sl[nsl - 1].y - sl[hnsl].y);
	for (int yb = sl[hnsl].y; yb >= sl[nsl - 1].y; yb--) {
		double p1 = -cam->fy * lsm->cv0 / 2;
		double p3 = cam->fy * cam->h;
		double p2 = cam->fy * lsm->phi2 + cam->cy - yb;
		
		double dist = -1;
		if (SolveQuadraticEquation(p1, p2, p3, &dist) || dist < 0) {
			continue;
		}
		
		double temp1 = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
		double yl = lsm->b / 2 + temp1;
		double temp2 = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
		int xb = (int)(cam->fx * yl / dist + temp2 + 0.5f);
		
		if (xb < 0 || xb > cam->xreso - 1) {
			continue;
		}
		
		int radius = (int)((yb - sl[hnsl].y) * k + sl[hnsl].radius);
		float direction = CaLaneDirection(cam, lsm, (float)dist, 1);
		ScanLine ssl = {xb, yb, radius, 0, 0, direction};
		ExtractTrackBorder(mag, dir, cam->xreso, cam->yreso, &ssl, 1);
		
		double yw = pww / 2 + temp1;
		int xpw = (int)(cam->fx * yw / dist + temp2 + 0.5f);
		
		yw = oww / 2 + temp1;
		int xow = (int)(cam->fx * yw / dist + temp2 + 0.5f);
		
		struct Track track{xb, yb, ssl.bx, xpw, xow, 1, (float)dist};
		TrackListAddTail(tracklist, &track);
	}
}*/
void CalTrackBoundary(float *mag, float *dir, Camera *cam, ScanLine *sl, int nsl, int side,
                      float pww, float oww, LaneStateModel *lsm, List *tracklist)
{	
	int s = (side == -1) ? 0 : 1;
	float k = (sl[nsl - 1].radius - sl[0].radius) / ((float)sl[nsl - 1].y - sl[0].y);
	for (int yb = sl[0].y; yb >= sl[nsl - 1].y; yb--) {
		double p1 = -cam->fy * lsm->cv0 / 2;
		double p3 = cam->fy * cam->h;
		double p2 = cam->fy * lsm->phi2 + cam->cy - yb;
		
		double dist = -1;
		if (SolveQuadraticEquation(p1, p2, p3, &dist) || dist < 0) {
			continue;
		}
		
		double temp1 = lsm->ch0 * pow(dist, 2) / 2 + lsm->ch1 * pow(dist, 3) / 6;
		double yl = side * lsm->b / 2 + temp1;
		double temp2 = cam->fx * (-lsm->y0 + lsm->phi1 * dist) / dist + cam->cx;
		int xb = (int)(cam->fx * yl / dist + temp2 + 0.5f);
		
		if (xb < 0 || xb > cam->xreso - 1) {
			continue;
		}
		
		int radius = (int)((yb - sl[0].y) * k + sl[0].radius);
		float direction = CaLaneDirection(cam, lsm, (float)dist, side);
		ScanLine ssl = {xb, yb, radius, 0, 0, direction};
		ExtractTrackBorder(mag, dir, cam->xreso, cam->yreso, &ssl, 1);

		double yw = side * pww / 2 + temp1;
		int xpw = (int)(cam->fx * yw / dist + temp2 + 0.5f);
		
		yw = side * oww / 2 + temp1;
		int xow = (int)(cam->fx * yw / dist + temp2 + 0.5f);
		
		struct Track *track = (struct Track *)ListAlloc(sizeof(struct Track));
		if (!track) exit(-1);
		
		track->px = xb;
		track->py = yb;
		track->mx = ssl.bx;
		track->pwx = xpw;
		track->owx = xow;
		track->side = s;
		track->dist = (float)dist;
		
		ListAddTail(tracklist, track);
	}
}

/** @brief Configuration update thread.
 ** @param param thread parameter.
 **/
void *ConfigUpdateThread(void *param)
{
	RailwayTracker *self = (RailwayTracker *)param;
	const char filename[] = "RailwayMonitor.json";
	time_t last_modify_time = time(NULL);
	struct stat file_stat_buf;
	
	while (self->runswitch) {
		if (!stat(filename, &file_stat_buf)) {
			if (last_modify_time != file_stat_buf.st_mtime) {
				InitLaneState("RailwayMonitor.json", &self->s0);
			}
			last_modify_time = file_stat_buf.st_mtime;
		}

		Sleep(1000);
	}
	
	return 0;
}

/** @brief Print matrix in command window.
 ** @param data matrix data.
 ** @param width matrix width.
 ** @param height matrix height.
 ** @param head head information.
 **/
void PrintMatrix(double *data, int width, int height, const char *head)
{
	cv::Mat mat(height, width, CV_64F, data);
	if (width > 1) {
		std::cout << head << mat << std::endl;
	} else {
		cv::Mat _mat;
		cv::transpose(mat, _mat);
		std::cout << head << _mat << std::endl;
	}
}

/** @brief Check current railway measurement point scan lines,
 **        and make a image under the running path.
 ** @param self railway tracker instance.
 ** @param bkgrnd background image.
 **/
void DrawTrackerScanLines(RailwayTracker *self, unsigned char *bkgrnd)
{
	// Create drawing image.
	cv::Mat BGR;
	if (bkgrnd) {
		cv::Mat _bkgrnd(self->camera.yreso, self->camera.xreso, CV_16UC1, bkgrnd);
				
		double minimum;
		double maximum;
		cv::minMaxLoc(_bkgrnd, &minimum, &maximum);
		
		cv::Mat norm = 255.0f * (_bkgrnd - minimum) / (maximum - minimum);
		
		cv::Mat u8norm;
		norm.convertTo(u8norm, CV_8UC1);
		
		cv::cvtColor(u8norm, BGR, CV_GRAY2BGR);
	} else {
		BGR.create(self->camera.yreso, self->camera.xreso, CV_8UC3);
		BGR.setTo(0);
	}
		
	int arrow = 5;
	for (int i = 0; i < self->measdim; i++) {
		// Scan line.
		cv::Point A(self->scanline[i].x - self->scanline[i].radius, self->scanline[i].y);
		cv::Point B(self->scanline[i].x + self->scanline[i].radius, self->scanline[i].y);
		cv::line(BGR, A, B, cv::Scalar(255, 255, 255));
		
		// Predicted lane boundary point.
		// cv::Point C(scanline[i].x, scanline[i].y);
		// int dx = (int)(arrow * cos(scanline[i].dir) + 0.5f);
		// int dy = (int)(arrow * sin(scanline[i].dir) + 0.5f);
		// cv::Point D(scanline[i].x + dx, scanline[i].y + dy);
		if (self->scanline[i].x >= 0 && self->scanline[i].x < self->camera.xreso) {
			cv::Point C(self->scanline[i].x, self->scanline[i].y - 3);
			cv::Point D(self->scanline[i].x, self->scanline[i].y + 3);
			cv::line(BGR, C, D, cv::Scalar(0, 255, 255));
		}
		
		// Extracted lane border point.
		if (self->scanline[i].bx >= 0 && self->scanline[i].bx < self->camera.xreso) {
			cv::Point E(self->scanline[i].bx, self->scanline[i].y - 3);
			cv::Point F(self->scanline[i].bx, self->scanline[i].y + 3);
			cv::line(BGR, E, F, cv::Scalar(0, 0, 255));
		}
		
		// State posteriori projected point.
		if (self->scanline[i].px >= 0 && self->scanline[i].px < self->camera.xreso) {
			cv::Point G(self->scanline[i].px, self->scanline[i].y - 2);
			cv::Point H(self->scanline[i].px, self->scanline[i].y + 2);
			cv::line(BGR, G, H, cv::Scalar(255, 0, 0));
		}
		
		// Cutoff line.
		if (fabs(cos(self->scanline[i].dir - self->scanline[i].edir)) < sqrt(3)/2) {
			int y = self->scanline[i].y;
			cv::line(BGR, cv::Point(0, y), cv::Point(BGR.cols - 1, y), cv::Scalar(0, 0, 0));
		}
	}

	char path[128];
	static int timer = 0;
	sprintf(path, "track\\%03d.png", timer++);
	cv::imwrite(path, BGR);
}

/** @brief Update cutoff line position.
 ** @param sl scan lines.
 ** @param nsl number of scan lines.
 ** @param bias_thresh predicted and measured direction bias threshold.
 ** @return the updated cutoff line position.
 **/
int UpdateCutoffLine(ScanLine *sl, int nsl, float bias_thresh)
{
	int cutoff = 0;
	for (int i = nsl / 2; i < nsl; i++) {
		if (fabs(cos(sl[i].dir - sl[i].edir)) < bias_thresh) {
			if (sl[i].y > cutoff) cutoff = sl[i].y;
		}
	}
	
	return cutoff;
}