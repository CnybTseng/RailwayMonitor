#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <io.h>
#include <windows.h>

#include "opencv2/opencv.hpp"
#include "HOGEngine.h"
#include "HOGImage.h"
#include "visualize.h"
// #include "frame.h"

#include "persondetectorwt.tcc"

#define PORT							(32345)
#define MAX_ORIGINAL_DATA_LENGTH		(1048576)

using namespace HOG;

HOGImage* image;
HOGImage* imageCUDA;

void print_help()
{
	printf("\n1. Detect pedestrian on infrared video stream.\n");
	printf("\tUsage: PersonDetector detect-infrared-video threshold\n");
	printf("2. Detect pedestrian on visual video stream.\n");
	printf("\tUsage: PersonDetector detect-visual-video threshold\n");
	printf("3. Detect pedestrian on dataset.\n");
	printf("\tUsage: PersonDetector detect-multiple-images scene_id start_id end_id threshold\n");
	printf("4. Detect pedestrian on single image.\n");
	printf("\tUsage: PersonDetector detect-single-image filename threshold\n");
}

cv::Mat read_image(char *filename,
                   bool normalized=false)
{
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		return image;
	}
	
	cv::Mat fltimage;
	
	if (!normalized) {
		image.convertTo(fltimage, CV_32FC1);
	} else {
		if (image.rows == WIN_HEI && image.cols == WIN_WID) {
			image.convertTo(fltimage, CV_32FC1);
		} else {
			cv::Mat rzimage;
			cv::resize(image, rzimage, cv::Size(WIN_WID, WIN_HEI), 0, 0, CV_INTER_AREA);
			rzimage.convertTo(fltimage, CV_32FC1);
		}
	}
	
	return fltimage;
}

void doStuffHere()
{
	double minimum, maximum;
	
	HOGEngine::Instance()->InitializeHOG(image->width, image->height,
			PERSON_LINEAR_BIAS, PERSON_WEIGHT_VEC, PERSON_WEIGHT_VEC_LENGTH);

	//HOGEngine::Instance()->InitializeHOG(image->width, image->height,
	//		"Files//SVM//head_W24x24_C4x4_N2x2_G4x4_HeadSize16x16.alt");

	// Timer t;
	// t.restart();
	clock_t start = clock();
	HOGEngine::Instance()->BeginProcess(image, 0, 0, image->width, image->height, 1, -1);
	HOGEngine::Instance()->EndProcess();
	clock_t finish = clock();
	// t.stop(); t.check("Processing time");

	printf("Taking %lfms, found %d positive results.\n", 1000.0f * (finish - start) / CLOCKS_PER_SEC,
		HOGEngine::Instance()->formattedResultsCount);

	HOGEngine::Instance()->GetImage(imageCUDA, HOGEngine::IMAGE_COLOR_GRADIENTS);
	// fastHOGWindow->setImage(imageCUDA);
	
	// cv::Mat imageOCV(imageCUDA->height, imageCUDA->width, CV_32FC1, imageCUDA->pixels);	
	// imageOCV = imageOCV(cv::Rect(10, 10, imageCUDA->width - 20, imageCUDA->height - 20));
	
	// cv::minMaxLoc(imageOCV, &minimum, &maximum);
	// printf("Gradient minmax: %lf %lf\n", minimum, maximum);
	
	// imageOCV = 255 * (imageOCV - minimum) / (maximum - minimum);
	// imageOCV.convertTo(imageOCV, CV_8UC1);
	// cv::imwrite("gradient.png", imageOCV);
	
	cv::Mat resultImage(image->height, image->width, CV_32FC1, image->pixels);
	cv::minMaxLoc(resultImage, &minimum, &maximum);
	resultImage = 255 * (resultImage - minimum) / (maximum - minimum);
	resultImage.convertTo(resultImage, CV_8UC1);
	cv::cvtColor(resultImage, resultImage, CV_GRAY2BGR);
	
	// cv::Mat resultImage(image->height, image->width, CV_8UC4, image->pixels);
	// cv::cvtColor(resultImage, resultImage, CV_RGBA2BGR);
	
	float thresh = 0.0f;

	for (int i=0; i<HOGEngine::Instance()->nmsResultsCount; i++)
	{
		printf("%1.5f %1.5f %4d %4d %4d %4d %4d %4d\n",
				HOGEngine::Instance()->nmsResults[i].scale,
				HOGEngine::Instance()->nmsResults[i].score,
				HOGEngine::Instance()->nmsResults[i].origX,
				HOGEngine::Instance()->nmsResults[i].origY,
				HOGEngine::Instance()->nmsResults[i].x,
				HOGEngine::Instance()->nmsResults[i].y,
				HOGEngine::Instance()->nmsResults[i].width,
				HOGEngine::Instance()->nmsResults[i].height);
				// fastHOGWindow->drawRect(HOGEngine::Instance()->nmsResults[i].x,
				//		HOGEngine::Instance()->nmsResults[i].y,
				//		HOGEngine::Instance()->nmsResults[i].width,
				//		HOGEngine::Instance()->nmsResults[i].height);
		
		if (HOGEngine::Instance()->nmsResults[i].score < thresh) continue;
		
		cv::rectangle(resultImage, cv::Rect(HOGEngine::Instance()->nmsResults[i].x,
			HOGEngine::Instance()->nmsResults[i].y, HOGEngine::Instance()->nmsResults[i].width,
			HOGEngine::Instance()->nmsResults[i].height), cv::Scalar(255, 255, 255));
		
		char text[64];
		sprintf_s(text, "%dx%d:%.2f", (int)(HOGEngine::Instance()->nmsResults[i].width),
			(int)(HOGEngine::Instance()->nmsResults[i].height), HOGEngine::Instance()->nmsResults[i].score);
		
		cv::Point txt_pt((int)(HOGEngine::Instance()->nmsResults[i].x),
			(int)(HOGEngine::Instance()->nmsResults[i].y) - 2);
		putText(resultImage, text, txt_pt, CV_FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, 8, false);	
	}

	printf("Drawn %d positive results.\n", HOGEngine::Instance()->nmsResultsCount);
	cv::imwrite("result.png", resultImage);
	HOGEngine::Instance()->FinalizeHOG();
}

void test_HOG_descriptor(int argc, char *argv[])
{
	assert(argc == 3);
	
	cv::Mat rawImage, fltimage;
	rawImage = cv::imread(argv[2], CV_LOAD_IMAGE_UNCHANGED);
	// rawImage = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	
	// int width = atoi(argv[3]);
	// int height = atoi(argv[4]);
	// rawImage.create(height, width, CV_16UC1);
	// FILE *ifp = fopen(argv[2], "rb");
	// assert(ifp);
	// fread(rawImage.data, sizeof(uint16_t), height * width, ifp);
	// fclose(ifp);
	
	rawImage.convertTo(fltimage, CV_32FC1);
	
	const int nCellsX = fltimage.cols / CELL_SIZE;
	const int nCellsY = fltimage.rows / CELL_SIZE;
	const int nBlocksX = nCellsX - BLK_CELL_SIZE + 1;
	const int nBlocksY = nCellsY - BLK_CELL_SIZE + 1;
	const int nDescCols = nBlocksX * BLK_CELL_SIZE * BLK_CELL_SIZE * NBINS;

	float *desc = new float[nBlocksY * nDescCols];
	
	cv::Mat gradient(fltimage.rows, fltimage.cols, CV_32FC1);
#if 0
	HOGEngine::Instance()->CalculateHOGDescriptorP((float *)fltimage.data, fltimage.cols, fltimage.rows,
		NBINS, fltimage.cols, fltimage.rows, CELL_SIZE, CELL_SIZE, BLK_CELL_SIZE, BLK_CELL_SIZE, desc, (float *)gradient.data);
#else
	HOGEngine::Instance()->InitHOGDescriptorCalculatorP(fltimage.cols, fltimage.rows,
		NBINS, fltimage.cols, fltimage.rows, CELL_SIZE, CELL_SIZE, BLK_CELL_SIZE, BLK_CELL_SIZE);
	
	HOGEngine::Instance()->HOGDescriptorCalculatorP((float *)fltimage.data, desc, (float *)gradient.data);
	
	HOGEngine::Instance()->FreeHOGDescriptorCalculatorP();
#endif
	
	gradient = gradient(cv::Rect(2, 2, gradient.cols - 4, gradient.rows - 4));

	double minimum, maximum;
	cv::minMaxLoc(gradient, &minimum, &maximum);
	printf("gradient %lf, %lf\n", minimum, maximum);
	gradient = 255 * (gradient - minimum) / (maximum - minimum);
	gradient.convertTo(gradient, CV_8UC1);
	cv::imwrite("gradient.png", gradient);
	
	cv::Mat blockHist = ToBlockHists(desc, nBlocksX, nBlocksY);
	
	cv::minMaxLoc(fltimage, &minimum, &maximum);
	fltimage = 255 * (fltimage - minimum) / (maximum - minimum);
	fltimage.convertTo(fltimage, CV_8UC1);
	
	cv::cvtColor(fltimage, fltimage, CV_GRAY2RGB);
	cv::imwrite("raw.png", fltimage);
	DrawHOG(fltimage, blockHist, nCellsX, nCellsY, 8, 4);
	cv::imwrite("descriptor.png", fltimage);
	
	FILE *fp;
	fopen_s(&fp, "descriptor.txt", "w");
	
	for (int i = 0; i < nBlocksY; i++) {
		for (int j = 0; j < nDescCols; j++) {
			fprintf(fp, "%f ", desc[i * nDescCols + j]);
		}
		fputs("\n", fp);
	}

	fclose(fp);
	
	if (desc) {
		delete [] desc;
		desc = NULL;
	}
}

/*void test_video_HOG_descriptor(int argc, char *argv[])
{
	char *data;
	float *desc = NULL;
	const int channel = 1;
	cv::Mat ushimage, fltimage;
	int keyboard = 0;
	uint16_t width, height;
	bool allocated = false;
	float scale = 1.0f;
	int move_up = 2;
	int nCellsX, nCellsY;
	int nBlocksX, nBlocksY;
	
	frame_receiver receiver(UDP_SERVER_USE, PORT);
	if (!receiver.open())
		return;
	
	if (!receiver.run())
		return;
	
	data = new char[MAX_ORIGINAL_DATA_LENGTH];
	assert(data);
	
	while ((char)keyboard != 'q' && (char)keyboard != 27) {
		if (receiver.get(channel, data, width, height)) {				
			if (false == allocated) {		
				ushimage.create(height, width, CV_16UC1);
				
				nCellsX = width / CELL_SIZE;
				nCellsY = height / CELL_SIZE;
				nBlocksX = nCellsX - BLK_CELL_SIZE + 1;
				nBlocksY = nCellsY - BLK_CELL_SIZE + 1;
				const int nDescCols = nBlocksX * BLK_CELL_SIZE * BLK_CELL_SIZE * NBINS;
				
				desc = new float[nBlocksY * nDescCols];
				
				HOGEngine::Instance()->InitHOGDescriptorCalculatorP(width, height,
					NBINS, width, height, CELL_SIZE, CELL_SIZE, BLK_CELL_SIZE, BLK_CELL_SIZE);
				
				allocated = true;
			}
			
			receiver.recombine_raw_data(data, (uint16_t *)ushimage.data, width * height * sizeof(uint16_t));	
			receiver.fill_zero_lines((uint16_t *)ushimage.data, width, height);
			
			ushimage.convertTo(fltimage, CV_32FC1);
		
			HOGEngine::Instance()->HOGDescriptorCalculatorP((float *)fltimage.data, desc);
			
			cv::Mat blockHist = ToBlockHists(desc, nBlocksX, nBlocksY);
			
			double minimum, maximum;
			cv::minMaxLoc(fltimage, &minimum, &maximum);
			fltimage = 255 * (fltimage - minimum) / (maximum - minimum);
			fltimage.convertTo(fltimage, CV_8UC1);
			
			cv::cvtColor(fltimage, fltimage, CV_GRAY2RGB);
			DrawHOG(fltimage, blockHist, nCellsX, nCellsY, 6, 1.5);
			
			// cv::rectangle(fltimage, cv::Rect(309 * 1.5, 89 * 1.5, 60 * 1.5, 121 * 1.5), cv::Scalar(255, 255, 255));
			// cv::rectangle(fltimage, cv::Rect(605 * 1.5, 175 * 1.5, 54 * 1.5, 108 * 1.5), cv::Scalar(255, 255, 255));
			// cv::rectangle(fltimage, cv::Rect(350 * 1.5, 255 * 1.5, 60 * 1.5, 120 * 1.5), cv::Scalar(255, 255, 255));
			
			cv::imshow("descriptor", fltimage);
			cv::imwrite("descriptor.png", fltimage);
			keyboard = cv::waitKey(10);
		}
	}
	
	receiver.stop();
	Sleep(1000);
	
	if (data != NULL) {
		delete [] data;
		data = NULL;
	}
	
	if (desc) {
		delete [] desc;
		desc = NULL;
	}
	
	if (allocated) {		
		HOGEngine::Instance()->FreeHOGDescriptorCalculatorP();
	}
}*/

void test_detect_single_image(int argc, char *argv[])
{
	assert(argc == 3);
	
	cv::Mat rawImage, fltimage;
	rawImage = cv::imread(argv[2], CV_LOAD_IMAGE_UNCHANGED);
	// rawImage = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	
	// rawImage.create(576, 720, CV_16UC1);
	// FILE *ifp = fopen(argv[2], "rb");
	// assert(ifp);
	// fread(rawImage.data, sizeof(uint16_t), 576 * 720, ifp);
	// fclose(ifp);
	
	rawImage.convertTo(fltimage, CV_32FC1);

	// cv::resize(fltimage, fltimage, cv::Size(fltimage.cols + 8, fltimage.rows + 8));
	
	// image = new HOGImage(argv[2]);
	image = new HOGImage(fltimage.cols, fltimage.rows, fltimage.data);
	
	imageCUDA = new HOGImage(image->width,image->height);
	// imageCUDA = new HOGImage(382,300);
	
	// fastHOGWindow = new ImageWindow(image, "fastHOG");
	// fastHOGWindow->doStuff = &doStuffHere;
	// fastHOGWindow->show();

	// fltk::run();
	doStuffHere();

	delete image;
	delete imageCUDA;
}

void get_files(string path, std::vector<string>& files)
{
	long hFile = 0;
	struct _finddata_t fileinfo;
	string p;
	if((hFile = _findfirst(p.assign(path).append("\\*").c_str(),&fileinfo)) != -1) {
		do {
			if ((fileinfo.attrib & _A_SUBDIR)) {
				if (strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
					get_files( p.assign(path).append("\\").append(fileinfo.name), files);
			} else {
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

void GetFilesUnderPath(const char *const cmd,
	                   const char *const format,
	                   std::vector<std::string> &filenames)
{
	assert(cmd);
	assert(format);
	
	enum {FILENAME_LENGTH = 128};
	
	// Command like 'dir /b /s Path'.
#ifdef _WIN32
	FILE *fp = _popen(cmd, "rt");
#elif __linux__
	FILE *fp = popen(cmd, "rt");
#else
#   error "Unknown compiler"
#endif

	if (!fp) {
		perror("popen");
		return;
	}

	char line_buf[FILENAME_LENGTH];
	char name[FILENAME_LENGTH];
	while (fgets(line_buf, FILENAME_LENGTH, fp)) {
		if (strstr(line_buf, format)) {	
			sscanf(line_buf, "%s\n", name);
			filenames.push_back(name);
		}
	}

#ifdef _WIN32	
	_pclose(fp);
#elif __linux__
	pclose(fp);
#else
#   error "Unknown compiler"
#endif
	fp = 0;
}

void save_detection_result(uint16_t *image,
                           int width,
						   int height,
						   int x,
						   int y,
						   int win_width,
						   int win_height,
						   char *dir)
{
	assert(image);
	
	cv::Mat mat(height, width, CV_16UC1, image);
	
	cv::Mat roi = mat(cv::Rect(x, y, win_width, win_height));
	
	cv::Mat window;
	roi.copyTo(window);
	
	cv::Mat norm_window;
	cv::resize(window, norm_window, cv::Size(WIN_WID, WIN_HEI));
	
	char filename[64];
	sprintf_s(filename, "%s\\counter.txt", dir);
	
	FILE *fp = fopen(filename, "r");
	assert(fp);
	
	int number = 0;
	fscanf(fp, "%d", &number);
	fclose(fp);
	
	sprintf(filename, "%s\\%05d.png", dir, number);
	
	std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
	
	cv::imwrite(filename, norm_window, compression_params);
	
	sprintf_s(filename, "%s\\counter.txt", dir);
	
	number++;
	fp = fopen(filename, "w");
	assert(fp);
	
	fprintf(fp, "%d", number);
	fclose(fp);
}

void test_dataset(int argc, char *argv[])
{
	if (argc < 8) {
		print_help();
		return;
	}
	
	bool allocated = false;
	const int scene = atoi(argv[3]);
	const int begin = atoi(argv[4]);
	const int end = atoi(argv[5]);
	float thresh = (float)atof(argv[6]);
	int flag = atoi(argv[7]);
	int keyboard = 0;
	float scale = 1.0f;
	int move_up = 2;
	float iouul = 0.5f;
	float ioull = 0.1f;
	
	cv::namedWindow("Result");
	HWND hWnd = (HWND)cvGetWindowHandle("Result");
	HWND hRawWnd = ::GetParent(hWnd);
	if (hRawWnd != NULL) {
		BOOL ret = ::SetWindowPos(hRawWnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOSIZE | SWP_NOMOVE);
		assert(ret);
	}
	
	cv::VideoWriter writer;
	char result_name[128];
	sprintf(result_name, "%s%d.avi", argv[2], scene);
	
	cv::Mat mask;
	cv::Mat bgrmask;
	enum {SAVE_HARD_SAMPLE = 1, SAVE_HARD_SAMPLE_ANYWAY};
	
	for (int i = begin; i <= end && (char)keyboard != 'q' && (char)keyboard != 27; i++) {
		char filename[128];
		sprintf_s(filename, "LSIFIR\\Detection\\%s\\%02d\\%05d.png", argv[2], scene, i);
		
		char annotation_filename[128];
		sprintf_s(annotation_filename, "LSIFIR\\Detection\\%s\\annotations\\%02d_%05d.txt", argv[2], scene, i);
		
		std::vector<location_t> locations = read_annotation(annotation_filename);
		
		for (size_t j = 0; j < locations.size(); j++) {
			printf("annotation %d: (%d, %d), (%d, %d)\n", j, locations[j].first.x, locations[j].first.y,
				locations[j].second.x, locations[j].second.y);
		}

		cv::Mat fltimage = read_image(filename);
		if (fltimage.empty()) break;
		
		if (false == allocated) {
			image = new HOGImage(fltimage.cols, fltimage.rows);
			
			HOGEngine::Instance()->InitializeHOG(fltimage.cols, fltimage.rows,
			PERSON_LINEAR_BIAS, PERSON_WEIGHT_VEC, PERSON_WEIGHT_VEC_LENGTH);
			
			writer.open(result_name, CV_FOURCC('M', 'J', 'P', 'G'), 25.0, fltimage.size());
			allocated = true;
			
			mask.create(fltimage.size(), CV_8UC1);
			mask.setTo(0);
			
			bgrmask.create(fltimage.size(), CV_8UC3);
			bgrmask.setTo(128);
		}
		
		memcpy(image->pixels, fltimage.data, fltimage.rows * fltimage.cols * sizeof(float));
		
		HOGEngine::Instance()->BeginProcess(image, 0, 0, image->width, image->height, 1.0f);
		HOGEngine::Instance()->EndProcess();
	
		double minimum, maximum;
		cv::minMaxLoc(fltimage, &minimum, &maximum);
		fltimage = 255 * (fltimage - minimum) / (maximum - minimum);
		
		cv::Mat uchimage;
		fltimage.convertTo(uchimage, CV_8UC1);
		
		cv::Mat rgbimage;
		cv::cvtColor(uchimage, rgbimage, CV_GRAY2BGR);
		
		for (size_t j = 0; j < locations.size(); j++) {
			dash_rectangle(rgbimage, 1, 2, cv::Rect(locations[j].first.x,
				locations[j].first.y, locations[j].second.x - locations[j].first.x + 1,
				locations[j].second.y - locations[j].first.y + 1), cv::Scalar(255, 255, 255), 1);
		}
		
		for (int i=0; i<HOGEngine::Instance()->nmsResultsCount; i++) {
			float iou = overlap(locations,
				HOGEngine::Instance()->nmsResults[i].x,
				HOGEngine::Instance()->nmsResults[i].y,
				HOGEngine::Instance()->nmsResults[i].x + HOGEngine::Instance()->nmsResults[i].width - 1,
				HOGEngine::Instance()->nmsResults[i].y + HOGEngine::Instance()->nmsResults[i].height - 1);
			
			printf("%1.5f %1.5f %4d %4d %4d %4d %4d %4d iou=%1.5f\n",
				HOGEngine::Instance()->nmsResults[i].scale,
				HOGEngine::Instance()->nmsResults[i].score,
				HOGEngine::Instance()->nmsResults[i].origX,
				HOGEngine::Instance()->nmsResults[i].origY,
				HOGEngine::Instance()->nmsResults[i].x,
				HOGEngine::Instance()->nmsResults[i].y,
				HOGEngine::Instance()->nmsResults[i].width,
				HOGEngine::Instance()->nmsResults[i].height,
				iou);

			if (flag) {
				if (iou > iouul) {
					cv::Mat ushimage = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
					save_detection_result((uint16_t *)ushimage.data, image->width, image->height,
						HOGEngine::Instance()->nmsResults[i].x, HOGEngine::Instance()->nmsResults[i].y,
						HOGEngine::Instance()->nmsResults[i].width, HOGEngine::Instance()->nmsResults[i].height,
						"TruePositive");
				}else if (iou < ioull) {
					cv::Mat roi = mask(cv::Rect(HOGEngine::Instance()->nmsResults[i].x,
						HOGEngine::Instance()->nmsResults[i].y,
						HOGEngine::Instance()->nmsResults[i].width,
						HOGEngine::Instance()->nmsResults[i].height));
					
					double ratset = cv::sum(roi)[0] / (roi.rows * roi.cols);
					if (SAVE_HARD_SAMPLE_ANYWAY == flag || ratset < 0.5) {
						cv::Mat ushimage = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
						save_detection_result((uint16_t *)ushimage.data, image->width, image->height,
							HOGEngine::Instance()->nmsResults[i].x, HOGEngine::Instance()->nmsResults[i].y,
							HOGEngine::Instance()->nmsResults[i].width, HOGEngine::Instance()->nmsResults[i].height,
							"FalsePositive");
						
						if (SAVE_HARD_SAMPLE == flag) {
							roi.setTo(1);
							cv::rectangle(bgrmask, cv::Rect(HOGEngine::Instance()->nmsResults[i].x,
							HOGEngine::Instance()->nmsResults[i].y,
							HOGEngine::Instance()->nmsResults[i].width,
							HOGEngine::Instance()->nmsResults[i].height), cv::Scalar(0, 255, 255));
						}
					}
				} else {
					;
				}
				
				if (SAVE_HARD_SAMPLE == flag) {
					cv::Mat _mask = mask.clone();
					_mask = _mask*255;
					imshow("seted", _mask);
					imshow("FP", bgrmask);
				}
			}
			
			if (HOGEngine::Instance()->nmsResults[i].score < thresh) continue;
				
			cv::rectangle(rgbimage, cv::Rect((int)(HOGEngine::Instance()->nmsResults[i].x * scale),
				(int)(HOGEngine::Instance()->nmsResults[i].y * scale),
				(int)(HOGEngine::Instance()->nmsResults[i].width * scale),
				(int)(HOGEngine::Instance()->nmsResults[i].height * scale)), cv::Scalar(255, 255, 255));
			
			char text[64];			
			sprintf_s(text, "S%.2f", HOGEngine::Instance()->nmsResults[i].score);
			cv::Point txt_pt((int)(HOGEngine::Instance()->nmsResults[i].x * scale),
				(int)(HOGEngine::Instance()->nmsResults[i].y * scale) - move_up);
			putText(rgbimage, text, txt_pt, CV_FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255, 255, 0), 1, 8, false);
		}
		
		if (HOGEngine::Instance()->nmsResultsCount > 0)
			printf("\n");
		
		writer << rgbimage;
		cv::imshow("Result", rgbimage);
		keyboard = cv::waitKey(1);
	}
	
	if (allocated) {
		delete image;		
		HOGEngine::Instance()->FinalizeHOG();
	}
}

void test_detect_binary_files(int argc, char *argv[])
{
	if (argc < 5) {
		print_help();
		return;
	}
	
	FILE *file_pointer = NULL;
	std::string filename = argv[2];
	
	float thresh = (float)atof(argv[3]);
	int flag = atoi(argv[4]);
	int keyboard = 0;
	float scale = 1.0f;
	int move_up = 2;
	
	fopen_s(&file_pointer, filename.c_str(), "rb");
	assert(file_pointer);

	std::vector<string> filenames;
	GetFilesUnderPath("dir /b /s samples", "dat", filenames);
	
	std::vector<string>::iterator file_iter;
	for (file_iter = filenames.begin(); file_iter != filenames.end(); file_iter++) {
		if (strstr(file_iter->c_str(), filename.c_str())) {
			break;
		}
	}
	
	enum {NFRAMES = 500};
	enum {HEIGHT = 480, WIDTH = 640};
	int timer = 0;
	bool allocated = false;
	
	cv::Mat mask;
	cv::Mat bgrmask;
	enum {SAVE_HARD_SAMPLE = 1, SAVE_HARD_SAMPLE_ANYWAY};
	
	while (27 != keyboard && 'q' != (char)keyboard) {
		if (++timer > NFRAMES) {
			fclose(file_pointer);
			file_pointer = NULL;
		
			if (file_iter == filenames.end()) {
				break;
			}
			
			file_iter++;
			if (file_iter == filenames.end()) {
				break;
			}
			
			fopen_s(&file_pointer, file_iter->c_str(), "rb");
			timer = 0;
		}

		cv::Mat frame(HEIGHT, WIDTH, CV_16UC1);
		fread_s(frame.data, WIDTH * HEIGHT * sizeof(uint16_t), sizeof(uint16_t), WIDTH * HEIGHT, file_pointer);
		
		cv::Mat fltimage;
		frame.convertTo(fltimage, CV_32FC1);
		
		if (false == allocated) {
			image = new HOGImage(fltimage.cols, fltimage.rows);			
			HOGEngine::Instance()->InitializeHOG(fltimage.cols, fltimage.rows,
			PERSON_LINEAR_BIAS, PERSON_WEIGHT_VEC, PERSON_WEIGHT_VEC_LENGTH);
			
			allocated = true;
			
			mask.create(fltimage.size(), CV_8UC1);
			mask.setTo(0);
			
			bgrmask.create(fltimage.size(), CV_8UC3);
			bgrmask.setTo(128);
		}

		memmove(image->pixels, fltimage.data, fltimage.rows * fltimage.cols * sizeof(float));
		
		clock_t start = clock();
		HOGEngine::Instance()->BeginProcess(image, 0, 0, image->width, image->height, -1, -1);
		HOGEngine::Instance()->EndProcess();
		clock_t finish = clock();
		printf("Processing %.0lfms\n", 1000.0f * (finish - start) / CLOCKS_PER_SEC);
		
		double minimum, maximum;
		cv::minMaxLoc(fltimage, &minimum, &maximum);
		fltimage = 255 * (fltimage - minimum) / (maximum - minimum);

		cv::Mat uchimage;
		fltimage.convertTo(uchimage, CV_8UC1);

		cv::Mat rgbimage;
		cv::cvtColor(uchimage, rgbimage, CV_GRAY2BGR);
		
		for (int i = 0; i < HOGEngine::Instance()->nmsResultsCount; i++) {
			printf("%1.5f %1.5f %4d %4d %4d %4d %4d %4d\n",
					HOGEngine::Instance()->nmsResults[i].scale,
					HOGEngine::Instance()->nmsResults[i].score,
					HOGEngine::Instance()->nmsResults[i].origX,
					HOGEngine::Instance()->nmsResults[i].origY,
					HOGEngine::Instance()->nmsResults[i].x,
					HOGEngine::Instance()->nmsResults[i].y,
					HOGEngine::Instance()->nmsResults[i].width,
					HOGEngine::Instance()->nmsResults[i].height);		
		
			if (HOGEngine::Instance()->nmsResults[i].score > thresh && flag && timer % 2 == 0) {
				cv::Mat roi = mask(cv::Rect(HOGEngine::Instance()->nmsResults[i].x,
					HOGEngine::Instance()->nmsResults[i].y,
					HOGEngine::Instance()->nmsResults[i].width,
					HOGEngine::Instance()->nmsResults[i].height));
					
				double ratset = cv::sum(roi)[0] / (roi.rows * roi.cols);
				if (SAVE_HARD_SAMPLE_ANYWAY == flag || ratset < 0.5) {
					save_detection_result((uint16_t *)frame.data,
						image->width, image->height, HOGEngine::Instance()->nmsResults[i].x,
						HOGEngine::Instance()->nmsResults[i].y, HOGEngine::Instance()->nmsResults[i].width,
						HOGEngine::Instance()->nmsResults[i].height, "FalsePositive");
					
					if (SAVE_HARD_SAMPLE == flag) {
						roi.setTo(1);
						cv::rectangle(bgrmask, cv::Rect(HOGEngine::Instance()->nmsResults[i].x,
						HOGEngine::Instance()->nmsResults[i].y,
						HOGEngine::Instance()->nmsResults[i].width,
						HOGEngine::Instance()->nmsResults[i].height), cv::Scalar(0, 255, 255));
					}
				}
				
				if (SAVE_HARD_SAMPLE == flag) {
					cv::Mat _mask = mask.clone();
					_mask = _mask*255;
					imshow("seted", _mask);
					imshow("FP", bgrmask);
				}
			}
		
			if (HOGEngine::Instance()->nmsResults[i].score < thresh) continue;
				
			cv::rectangle(rgbimage, cv::Rect((int)(HOGEngine::Instance()->nmsResults[i].x * scale),
				(int)(HOGEngine::Instance()->nmsResults[i].y * scale),
				(int)(HOGEngine::Instance()->nmsResults[i].width * scale),
				(int)(HOGEngine::Instance()->nmsResults[i].height * scale)), cv::Scalar(255, 255, 255));
			
			char text[64];			
			sprintf_s(text, "S%.2f", HOGEngine::Instance()->nmsResults[i].score);
			cv::Point txt_pt((int)(HOGEngine::Instance()->nmsResults[i].x * scale),
				(int)(HOGEngine::Instance()->nmsResults[i].y * scale) - move_up);
			putText(rgbimage, text, txt_pt, CV_FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(255, 255, 0), 1, 8, false);
		}
		
		if (HOGEngine::Instance()->nmsResultsCount > 0)
			printf("\n");
		
		cv::imshow("Result", rgbimage);
		keyboard = cv::waitKey(1);
	}
	
	if (allocated) {
		delete image;	
		HOGEngine::Instance()->FinalizeHOG();
	}
}

void test_detect_vs_video(int argc, char *argv[])
{
	cv::Mat rgbimage, imageRGBA, uchimage, fltimage;
	int keyboard = 0;
	bool allocated = false;
	float thresh = (float)atof(argv[3]);
	float scale = 1;
	int move_up = 2;
	
	cv::VideoCapture cap(argv[2]);
	assert(cap.isOpened());

	cv::namedWindow("Result");
	HWND hWnd = (HWND)cvGetWindowHandle("Result");
	HWND hRawWnd = ::GetParent(hWnd);
	if (hRawWnd != NULL) {
		BOOL ret = ::SetWindowPos(hRawWnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOSIZE | SWP_NOMOVE);
		assert(ret);
	}
	
	while ((char)keyboard != 'q' && (char)keyboard != 27) {
		cap >> rgbimage;
		
		if (!rgbimage.empty()) {				
			if (false == allocated) {						
				image = new HOGImage(rgbimage.cols, rgbimage.rows);
				imageCUDA = new HOGImage(rgbimage.cols, rgbimage.rows);
				
				HOGEngine::Instance()->InitializeHOG(rgbimage.cols, rgbimage.rows,
				PERSON_LINEAR_BIAS, PERSON_WEIGHT_VEC, PERSON_WEIGHT_VEC_LENGTH);
				
				allocated = true;
			}
			
			// cv::cvtColor(rgbimage, uchimage, CV_BGR2GRAY);
			// uchimage.convertTo(fltimage, CV_32FC1);
			
			cv::cvtColor(rgbimage, imageRGBA, CV_BGR2RGBA);
			
			// memcpy(image->pixels, fltimage.data, fltimage.rows * fltimage.cols * sizeof(float));
			memcpy(image->pixels, imageRGBA.data, imageRGBA.rows * imageRGBA.cols * 4);
		
			clock_t start = clock();
			HOGEngine::Instance()->BeginProcess(image, 0, 0, image->width, image->height, 1, -1);
			clock_t finish = clock();
			
			printf("Detection:%.0lfms, ", 1000.0f * (finish - start) / CLOCKS_PER_SEC);
			
			start = clock();
			HOGEngine::Instance()->EndProcess();
			finish = clock();
			
			printf("mergeing:%.0lfms, total positive:%d.\n", 1000.0f * (finish - start) / CLOCKS_PER_SEC,
				HOGEngine::Instance()->formattedResultsCount);

			for (int i=0; i<HOGEngine::Instance()->nmsResultsCount; i++)
			{
				printf("%1.5f %1.5f %4d %4d %4d %4d %4d %4d\n",
						HOGEngine::Instance()->nmsResults[i].scale,
						HOGEngine::Instance()->nmsResults[i].score,
						HOGEngine::Instance()->nmsResults[i].origX,
						HOGEngine::Instance()->nmsResults[i].origY,
						HOGEngine::Instance()->nmsResults[i].x,
						HOGEngine::Instance()->nmsResults[i].y,
						HOGEngine::Instance()->nmsResults[i].width,
						HOGEngine::Instance()->nmsResults[i].height);
				
				if (HOGEngine::Instance()->nmsResults[i].score < thresh) continue;
					
				cv::rectangle(imageRGBA, cv::Rect((int)(HOGEngine::Instance()->nmsResults[i].x),
					(int)(HOGEngine::Instance()->nmsResults[i].y), (int)(HOGEngine::Instance()->nmsResults[i].width),
					(int)(HOGEngine::Instance()->nmsResults[i].height)), cv::Scalar(0, 255, 255), 2);
				
				char text[64];
				// sprintf_s(text, "%dx%d:%.2f", (int)(HOGEngine::Instance()->nmsResults[i].width),
				//	(int)(HOGEngine::Instance()->nmsResults[i].height), HOGEngine::Instance()->nmsResults[i].score);
				
				sprintf_s(text, "S%.2f", HOGEngine::Instance()->nmsResults[i].score);
				
				cv::Point txt_pt((int)(HOGEngine::Instance()->nmsResults[i].x),
					(int)(HOGEngine::Instance()->nmsResults[i].y) - move_up);
				putText(imageRGBA, text, txt_pt, CV_FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1, 8, false);	
			}
			
			printf("\n");
			cv::imwrite("Result.png", imageRGBA);
			cv::resize(imageRGBA, imageRGBA, cv::Size(imageRGBA.cols / 3, imageRGBA.rows / 3));

			cv::imshow("Result", imageRGBA);
			keyboard = cv::waitKey(10);
		} else {
			break;
		}
	}
	
	if (allocated) {
		delete image;
		delete imageCUDA;
		
		HOGEngine::Instance()->FinalizeHOG();
	}
}

/*void test_detect_ir_video(int argc, char *argv[])
{
	char *data;
	const int channel = 1;
	cv::Mat ushimage, fltimage;
	int keyboard = 0;
	uint16_t width, height;
	bool allocated = false;
	float thresh = (float)atof(argv[2]);
	float scale = 1;
	int move_up = 2;
	double minimum, maximum;
	
	frame_receiver receiver(UDP_SERVER_USE, PORT);
	if (!receiver.open())
		return;
	
	if (!receiver.run())
		return;
	
	data = new char[MAX_ORIGINAL_DATA_LENGTH];
	assert(data);

	cv::namedWindow("Result");
	HWND hWnd = (HWND)cvGetWindowHandle("Result");
	HWND hRawWnd = ::GetParent(hWnd);
	if (hRawWnd != NULL) {
		BOOL ret = ::SetWindowPos(hRawWnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOSIZE | SWP_NOMOVE);
		assert(ret);
	}
	
	while ((char)keyboard != 'q' && (char)keyboard != 27) {
		if (receiver.get(channel, data, width, height)) {				
			if (false == allocated) {			
				ushimage.create(height, width, CV_16UC1);
				
				image = new HOGImage(width, height);
				imageCUDA = new HOGImage(width, height);
				
				HOGEngine::Instance()->InitializeHOG(width, height,
				PERSON_LINEAR_BIAS, PERSON_WEIGHT_VEC, PERSON_WEIGHT_VEC_LENGTH);
				
				allocated = true;
			}
			
			receiver.recombine_raw_data(data, (uint16_t *)ushimage.data, width * height * sizeof(uint16_t));	
			receiver.fill_zero_lines((uint16_t *)ushimage.data, width, height);
			
			ushimage.convertTo(fltimage, CV_32FC1);
			
			memcpy(image->pixels, fltimage.data, fltimage.rows * fltimage.cols * sizeof(float));
		
			clock_t start = clock();
			HOGEngine::Instance()->BeginProcess(image, 0, 0, image->width, image->height, 1, -1);
			clock_t finish = clock();
			
			printf("Detection:%.0lfms, ", 1000.0f * (finish - start) / CLOCKS_PER_SEC);
			
			start = clock();
			HOGEngine::Instance()->EndProcess();
			finish = clock();
			
			printf("mergeing:%.0lfms, total positive:%d.\n", 1000.0f * (finish - start) / CLOCKS_PER_SEC,
				HOGEngine::Instance()->formattedResultsCount);
		
			cv::minMaxLoc(fltimage, &minimum, &maximum);
			fltimage = 255 * (fltimage - minimum) / (maximum - minimum);
			
			cv::Mat uchimage;
			fltimage.convertTo(uchimage, CV_8UC1);
			
			cv::Mat rgbimage;
			cv::cvtColor(uchimage, rgbimage, CV_GRAY2BGR);

			for (int i=0; i<HOGEngine::Instance()->nmsResultsCount; i++)
			{
				printf("%1.5f %1.5f %4d %4d %4d %4d %4d %4d\n",
						HOGEngine::Instance()->nmsResults[i].scale,
						HOGEngine::Instance()->nmsResults[i].score,
						HOGEngine::Instance()->nmsResults[i].origX,
						HOGEngine::Instance()->nmsResults[i].origY,
						HOGEngine::Instance()->nmsResults[i].x,
						HOGEngine::Instance()->nmsResults[i].y,
						HOGEngine::Instance()->nmsResults[i].width,
						HOGEngine::Instance()->nmsResults[i].height);
				
				if (HOGEngine::Instance()->nmsResults[i].score < thresh) continue;
					
				cv::rectangle(rgbimage, cv::Rect((int)(HOGEngine::Instance()->nmsResults[i].x),
					(int)(HOGEngine::Instance()->nmsResults[i].y), (int)(HOGEngine::Instance()->nmsResults[i].width),
					(int)(HOGEngine::Instance()->nmsResults[i].height)), cv::Scalar(0, 255, 255), 2);
				
				char text[64];
				// sprintf_s(text, "%dx%d:%.2f", (int)(HOGEngine::Instance()->nmsResults[i].width),
				//	(int)(HOGEngine::Instance()->nmsResults[i].height), HOGEngine::Instance()->nmsResults[i].score);
				
				sprintf_s(text, "S%.2f", HOGEngine::Instance()->nmsResults[i].score);
				
				cv::Point txt_pt((int)(HOGEngine::Instance()->nmsResults[i].x),
					(int)(HOGEngine::Instance()->nmsResults[i].y) - move_up);
				putText(rgbimage, text, txt_pt, CV_FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1, 8, false);	
				
				save_detection_result((uint16_t *)ushimage.data, image->width, image->height, HOGEngine::Instance()->nmsResults[i].x,
					HOGEngine::Instance()->nmsResults[i].y, HOGEngine::Instance()->nmsResults[i].width,
					HOGEngine::Instance()->nmsResults[i].height);
			}
			
			printf("\n");
			cv::imwrite("Result.png", rgbimage);
			// cv::resize(rgbimage, rgbimage, cv::Size(width / 3, height / 3));

			cv::imshow("Result", rgbimage);
			keyboard = cv::waitKey(10);
		}
	}
	
	receiver.stop();
	Sleep(1000);
	
	if (data != NULL) {
		delete [] data;
		data = NULL;
	}
	
	if (allocated) {
		delete image;
		delete imageCUDA;
		
		HOGEngine::Instance()->FinalizeHOG();
	}
}*/ 

int main(int argc, char *argv[])
{
	if (argc == 1) {
		print_help();
		return 0;
	}

	if (!strcmp(argv[1], "detect-single-image")) {
		test_detect_single_image(argc, argv);
	} else if (!strcmp(argv[1], "detect-dataset")) {
		test_dataset(argc, argv);
	} else if (!strcmp(argv[1], "detect-visual-video")) {
		test_detect_vs_video(argc, argv);
	} else if (!strcmp(argv[1], "detect-infrared-video")) {
		// test_detect_ir_video(argc, argv);
	} else if (!strcmp(argv[1], "hog")) {
		test_HOG_descriptor(argc, argv);
	} else if (!strcmp(argv[1], "detect_binary_files")) {
		test_detect_binary_files(argc, argv);
	}

	return 0;
}