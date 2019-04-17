#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"

void main(int argc, char *argv[])
{
	if (argc <= 1) {
		fprintf(stderr, "please enter a 640x480 image.\n");
		exit(-1);
	}
	
	cv::Mat image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
	if (image.empty()) {
		fprintf(stderr, "cv::imread fail.\n");
		exit(-1);
	}
	
	cv::Mat u16_image;
	image.convertTo(u16_image, CV_16UC1);
	
	FILE *fp = fopen("static_scene.dat", "wb");
	if (!fp) {
		fprintf(stderr, "fopen fail.\n");
		exit(-1);
	}
	
	for (int i = 0; i < 500; ++i) {
		fwrite((unsigned short *)u16_image.data, sizeof(unsigned short), u16_image.rows * u16_image.cols, fp);
	}
	
	fclose(fp);
}