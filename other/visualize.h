#ifndef _VISUALIZE_H_
#define _VISUALIZE_H_

#include <stdint.h>
#include "opencv2/opencv.hpp"

#define HOG_DEBUG 						(0)
#define RAD_PER_ANG						(0.0174532925167)
#define ANG_PER_RAD						(57.295779523839)
#define WIN_WID							(16)
#define WIN_HEI							(32)
#define CELL_SIZE						(4)
#define NBINS							(9)
#define ANG_STEP						(20)
#define BLK_CELL_SIZE					(2)

#define CLASSIFY_PATH					"LSIFIR\\Classification\\"
#define TRAIN_POS_NUM					(10208)
#define TRAIN_NEG_NUM					(43390)
#define TEST_POS_NUM					(5944)
#define TEST_NEG_NUM					(22050)

typedef std::pair<cv::Point, cv::Point> location_t;

cv::Mat ToBlockHists(float *hostHog,
                     int nBlocksX,
					 int nBlocksY);
					 
void DrawHOG(cv::Mat &image,
	         cv::Mat blockHists,
	         uint32_t xcells,
	         uint32_t ycells,
	         uint32_t lenLmt,
	         float scale);					 

std::vector<location_t> read_annotation(char *filename);

float overlap(std::vector<location_t> location,
              int tlcx,
			  int tlcy,
			  int lrcx,
			  int lrcy);					

void dash_rectangle(cv::Mat img,
                    int linelength,
					int dashlength,
					cv::Rect blob,
					cv::Scalar color,
					int thickness);
							
#endif