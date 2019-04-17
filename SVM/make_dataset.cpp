
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "opencv2/opencv.hpp"
#include "HOGEngine.h"

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

static void print_help();
static uint32_t get_augmented_dataset_size();
static void make_train_dataset();
static void make_test_dataset();
static void write_HOG_to_text_line(float *hog, int len, int label, FILE *fp);
static cv::Mat read_image(char *filename, bool normalized = false);

int main(int argc, char *argv[])
{
	if (argc == 1) {
		print_help();
		return EXIT_FAILURE;
	}

	if (!strcmp(argv[1], "mktrain")) {
		make_train_dataset();
	} else if (!strcmp(argv[1], "mktest")) {
		make_test_dataset();
	} else {
		fprintf(stderr, "unknown command!\n");
		return  EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

void print_help()
{
	printf("\n1. Make training or testing dataset.\n");
	printf("\tUsage: MakeDataset mktrain\n");
	printf("\t       MakeDataset mktest\n");
}

uint32_t get_augmented_dataset_size()
{
	FILE *fp = fopen("FalsePositive\\counter.txt", "r");
	if (!fp) {
		fprintf(stderr, "Not found augmented dataset!\n");
		return 0;
	}
	
	int number = 0;
	fscanf(fp, "%d", &number);
	fclose(fp);
	
	return number;
}

void make_train_dataset()
{
	FILE *fp = NULL;
	fopen_s(&fp, "train.txt", "w");
	assert(fp);
	
	int nCellsX = WIN_WID / CELL_SIZE;
	int nCellsY = WIN_HEI / CELL_SIZE;
	int nBlocksX = nCellsX - BLK_CELL_SIZE + 1;
	int nBlocksY = nCellsY - BLK_CELL_SIZE + 1;
	
	const int hog_len = nBlocksX * nBlocksY * BLK_CELL_SIZE * BLK_CELL_SIZE * NBINS;
	float *hostHog = new float[hog_len];
	
	cv::Mat gradient(WIN_HEI, WIN_WID, CV_32FC1);

	HOG::HOGEngine::Instance()->InitHOGDescriptorCalculatorP(WIN_WID, WIN_HEI,
		NBINS, WIN_WID, WIN_HEI, CELL_SIZE, CELL_SIZE, BLK_CELL_SIZE, BLK_CELL_SIZE);
	
	printf("Make train positive dataset...");
	for (uint32_t i = 0; i < TRAIN_POS_NUM; i++) {
		char positive_sample_file[128];
		sprintf_s(positive_sample_file, "LSIFIR\\Classification\\Train\\pos\\%05d.png", i);

		cv::Mat imageF32 = read_image(positive_sample_file, true);
		if (imageF32.empty())
			continue;
		
		HOG::HOGEngine::Instance()->HOGDescriptorCalculatorP((float *)imageF32.data, hostHog);
		
		write_HOG_to_text_line(hostHog, hog_len, 1, fp);

		if (i % 200 == 0) {
			printf("%u..", i);
		}
	}
	printf("done\nMake train negative dataset...");
	for (uint32_t i = 0; i < TRAIN_NEG_NUM; i++) {
		char negative_sample_file[128];
		sprintf_s(negative_sample_file, "LSIFIR\\Classification\\Train\\neg\\%05d.png", i);
		
		cv::Mat imageF32 = read_image(negative_sample_file, true);
		if (imageF32.empty())
			continue;
		
		HOG::HOGEngine::Instance()->HOGDescriptorCalculatorP((float *)imageF32.data, hostHog);
		
		write_HOG_to_text_line(hostHog, hog_len, -1, fp);

		if (i % 200 == 0) {
			printf("%u..", i);
		}
	}
	printf("done\nMake augmented train negative dataset...");
	uint32_t number = get_augmented_dataset_size();
	for (uint32_t i = 0; i < number; i++) {
		char negative_sample_file[128];
		sprintf_s(negative_sample_file, "FalsePositive\\%05d.png", i);
		
		cv::Mat imageF32 = read_image(negative_sample_file, true);
		if (imageF32.empty())
			continue;
		
		HOG::HOGEngine::Instance()->HOGDescriptorCalculatorP((float *)imageF32.data, hostHog);
		
		write_HOG_to_text_line(hostHog, hog_len, -1, fp);

		if (i % 200 == 0) {
			printf("%u..", i);
		}
	}
	printf("done\n");
	fclose(fp);
	delete [] hostHog;
	HOG::HOGEngine::Instance()->FreeHOGDescriptorCalculatorP();
}

void make_test_dataset()
{
	FILE *fp = NULL;
	fopen_s(&fp, "test.txt", "w");
	assert(fp);
	
	int nCellsX = WIN_WID / CELL_SIZE;
	int nCellsY = WIN_HEI / CELL_SIZE;
	int nBlocksX = nCellsX - BLK_CELL_SIZE + 1;
	int nBlocksY = nCellsY - BLK_CELL_SIZE + 1;
	
	const int hog_len = nBlocksX * nBlocksY * BLK_CELL_SIZE * BLK_CELL_SIZE * NBINS;
	float *hostHog = new float[hog_len];

	cv::Mat gradient(WIN_HEI, WIN_WID, CV_32FC1);
	
	HOG::HOGEngine::Instance()->InitHOGDescriptorCalculatorP(WIN_WID, WIN_HEI,
		NBINS, WIN_WID, WIN_HEI, CELL_SIZE, CELL_SIZE, BLK_CELL_SIZE, BLK_CELL_SIZE);

	printf("Make test positive dataset...");
	for (uint32_t i = 0; i < TEST_POS_NUM; i++) {
		char positive_sample_file[128];
		sprintf_s(positive_sample_file, "LSIFIR\\Classification\\Test\\pos\\%05d.png", i);		
		
		cv::Mat imageF32 = read_image(positive_sample_file, true);
		if (imageF32.empty())
			continue;
		
		HOG::HOGEngine::Instance()->HOGDescriptorCalculatorP((float *)imageF32.data, hostHog);
		
		write_HOG_to_text_line(hostHog, hog_len, 1, fp);
		
		if (i % 200 == 0) {
			printf("%u..", i);
		}
	}
	printf("done\nMake test negative dataset...");
	for (uint32_t i = 0; i < TEST_NEG_NUM; i++) {
		char negative_sample_file[128];
		sprintf_s(negative_sample_file, "LSIFIR\\Classification\\Test\\neg\\%05d.png", i);
		
		cv::Mat imageF32 = read_image(negative_sample_file, true);
		if (imageF32.empty())
			continue;
		
		HOG::HOGEngine::Instance()->HOGDescriptorCalculatorP((float *)imageF32.data, hostHog);
		
		write_HOG_to_text_line(hostHog, hog_len, -1, fp);

		if (i % 200 == 0) {
			printf("%u..", i);
		}
	}
	printf("done\n");
	fclose(fp);
	delete [] hostHog;
	HOG::HOGEngine::Instance()->FreeHOGDescriptorCalculatorP();
}

void write_HOG_to_text_line(float *hog, int len, int label, FILE *fp)
{
	assert(hog);
	assert(fp);
	
	fprintf(fp, "%d ", label);
	
	for (int i = 0; i < len; i++) {
		fprintf(fp, "%d:%f ", i + 1, hog[i]);
	}
	
	fputs("\n", fp);
}

cv::Mat read_image(char *filename, bool normalized)
{
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		return image;
	}
	
	cv::Mat imageF32;
	
	if (!normalized) {
		image.convertTo(imageF32, CV_32FC1);
	} else {
		if (image.rows == WIN_HEI && image.cols == WIN_WID) {
			image.convertTo(imageF32, CV_32FC1);
		} else {
			cv::Mat rzimage;
			cv::resize(image, rzimage, cv::Size(WIN_WID, WIN_HEI), 0, 0, CV_INTER_AREA);
			rzimage.convertTo(imageF32, CV_32FC1);
		}
	}
	
	return imageF32;
}