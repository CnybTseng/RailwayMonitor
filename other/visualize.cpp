#include "HOGEngine.h"
#include "visualize.h"

cv::Mat ToBlockHists(float *hostHog,
                     int nBlocksX,
					 int nBlocksY)
{
	assert(hostHog);
	
	cv::Mat blockHists(nBlocksX * nBlocksY, BLK_CELL_SIZE * BLK_CELL_SIZE * NBINS, CV_32FC1);

	int blockStride = NBINS * BLK_CELL_SIZE * nBlocksX * BLK_CELL_SIZE;

	for (int i = 0; i < nBlocksX * nBlocksY; i++) {
		float *ptr = hostHog + (i / nBlocksX) * blockStride + (i % nBlocksX) * NBINS * BLK_CELL_SIZE;
		for (int x = 0; x < 2 * NBINS; x++) {
			blockHists.ptr<float>(i)[x] = ptr[x];
		}
		ptr += (blockStride / 2);
		for (int x = 2 * NBINS; x < 4 * NBINS; x++) {
			blockHists.ptr<float>(i)[x] = ptr[x - 2 * NBINS];
		}
	}

	return blockHists;
}

void DrawHOG(cv::Mat &image,
	         cv::Mat blockHists,
	         uint32_t xcells,
	         uint32_t ycells,
	         uint32_t lenLmt,
	         float scale)
{
	assert(!image.empty());
	assert(!blockHists.empty());

	const uint32_t block_bins = BLK_CELL_SIZE * BLK_CELL_SIZE * NBINS;
	uint32_t block_id = 0;

	cv::resize(image, image, cv::Size(0, 0), scale, scale);

	for (uint32_t y = 0; y < ycells - 1; y++) {
		// Draw HOG of first cell.
		for (uint32_t x = 0; x < xcells - 1; x++) {
			cv::Point center((int)((CELL_SIZE / 2 + x * CELL_SIZE) * scale), (int)((CELL_SIZE / 2 + y * CELL_SIZE) * scale));
			float *pblock = (float *)blockHists.data + block_id * block_bins;

			float maximum = 0;
			for (uint32_t i = 0; i < NBINS; i++) {
				if (pblock[i] > maximum) {
					maximum = pblock[i];
				}
			}

			for (uint32_t i = 0; i < NBINS; i++) {
				float len = lenLmt * pblock[i] / maximum;
				float angle = (float)i * ANG_STEP;

				if (angle < 90) {
					angle += 90;
				} else {
					angle -= 90;
				}

				angle = (float)(angle * RAD_PER_ANG);
				cv::Point start((int)(center.x - len * cos(angle)), (int)(center.y - len * sin(angle)));
				cv::Point end((int)(center.x + len * cos(angle)), (int)(center.y + len * sin(angle)));
				cv::line(image, start, end, cv::Scalar(0, 255, 255));
			}
			block_id++;
		}

		// Draw HOG of second cell.
		const uint32_t x = xcells - 1;
		cv::Point center((int)((CELL_SIZE / 2 + x * CELL_SIZE) * scale), (int)((CELL_SIZE / 2 + y * CELL_SIZE) * scale));
		float *pblock = (float *)blockHists.data + (block_id - 1) * block_bins;

		float maximum = 0;
		for (uint32_t i = NBINS; i < 2 * NBINS; i++) {
			if (pblock[i] > maximum) {
				maximum = pblock[i];
			}
		}

		for (uint32_t i = NBINS; i < 2 * NBINS; i++) {
			float len = lenLmt * pblock[i] / maximum;
			float angle = (float)((i - NBINS) * ANG_STEP);

			if (angle < 90) {
				angle += 90;
			} else {
				angle -= 90;
			}

			angle = (float)(angle * RAD_PER_ANG);
			cv::Point start((int)(center.x - len * cos(angle)), (int)(center.y - len * sin(angle)));
			cv::Point end((int)(center.x + len * cos(angle)), (int)(center.y + len * sin(angle)));
			cv::line(image, start, end, cv::Scalar(0, 255, 255));
		}
	}

	// Draw HOG of third cell.
	const uint32_t y = ycells - 1;
	block_id -= xcells;
	for (uint32_t x = 0; x < xcells - 1; x++) {
		cv::Point center((int)((CELL_SIZE / 2 + x * CELL_SIZE) * scale), (int)((CELL_SIZE / 2 + y * CELL_SIZE) * scale));
		float *pblock = (float *)blockHists.data + block_id * block_bins;

		float maximum = 0;
		for (uint32_t i = 2 * NBINS; i < 3 * NBINS; i++) {
			if (pblock[i] > maximum) {
				maximum = pblock[i];
			}
		}

		for (uint32_t i = 2 * NBINS; i < 3 * NBINS; i++) {
			float len = lenLmt * pblock[i] / maximum;
			float angle = (float)((i - 2 * NBINS) * ANG_STEP);

			if (angle < 90) {
				angle += 90;
			} else {
				angle -= 90;
			}

			angle = (float)(angle * RAD_PER_ANG);
			cv::Point start((int)(center.x - len * cos(angle)), (int)(center.y - len * sin(angle)));
			cv::Point end((int)(center.x + len * cos(angle)), (int)(center.y + len * sin(angle)));
			cv::line(image, start, end, cv::Scalar(0, 255, 255));
		}
		block_id++;
	}

	// Draw HOG of fourth cell.
	block_id--;
	const uint32_t x = xcells - 1;
	cv::Point center((int)((CELL_SIZE / 2 + x * CELL_SIZE) * scale), (int)((CELL_SIZE / 2 + y * CELL_SIZE) * scale));
	float *pblock = (float *)blockHists.data + block_id * block_bins;

	float maximum = 0;
	for (uint32_t i = 3 * NBINS; i < 4 * NBINS; i++) {
		if (pblock[i] > maximum) {
			maximum = pblock[i];
		}
	}

	for (uint32_t i = 3 * NBINS; i < 4 * NBINS; i++) {
		float len = lenLmt * pblock[i] / maximum;
		float angle = (float)((i - 3 * NBINS) * ANG_STEP);

		if (angle < 90) {
			angle += 90;
		} else {
			angle -= 90;
		}

		angle = (float)(angle * RAD_PER_ANG);
		cv::Point start((int)(center.x - len * cos(angle)), (int)(center.y - len * sin(angle)));
		cv::Point end((int)(center.x + len * cos(angle)), (int)(center.y + len * sin(angle)));
		cv::line(image, start, end, cv::Scalar(0, 255, 255));
	}
}

std::vector<location_t> read_annotation(char *filename)
{
	std::vector<location_t> location;
	
	FILE *fp = fopen(filename, "r");
	if (NULL == fp)
		return location;
	
	while (!feof(fp)) {
		char line_buf[128] = {0};
		fgets(line_buf, sizeof(line_buf), fp);
		if (strstr(line_buf, "Bounding")) {
			char *ptr = strstr(line_buf, ":");
			int tlcx, tlcy, lrcx, lrcy;
			sscanf(ptr, ": (%d, %d) - (%d, %d)\n", &tlcx, &tlcy, &lrcx, &lrcy);
			location.push_back(std::pair<cv::Point, cv::Point>(cv::Point(tlcx, tlcy), cv::Point(lrcx, lrcy)));
		}
	}
	
	fclose(fp);
	return location;
}

float overlap(std::vector<location_t> location,
              int tlcx,
			  int tlcy,
			  int lrcx,
			  int lrcy)
{
	if (location.size() <= 0)
		return 0;
	
	std::vector<location_t>::iterator it;
	float olp_max = 0;
	
	for (it = location.begin(); it != location.end(); it++) {			
		int32_t olx = std::max(it->first.x, tlcx);
		int32_t oty = std::max(it->first.y, tlcy);
		
		int32_t orx = std::min(it->second.x, lrcx);
		int32_t oby = std::min(it->second.y, lrcy);
		
		int32_t w = orx - olx + 1;
		int32_t h = oby - oty + 1;
		if (w > 0 && h > 0) {	
			int ground_truth_area = (it->second.x - it->first.x + 1) * (it->second.y - it->first.y + 1);
			int detection_area = (lrcx - tlcx + 1) * (lrcy - tlcy + 1);
			float oa = (float)w * h / (ground_truth_area + detection_area - w * h);

			if (oa > olp_max) {
				olp_max = oa;
			}
		}
	}
	
	return olp_max;
}

void dash_rectangle(cv::Mat img,
                    int linelength,
					int dashlength,
					cv::Rect blob,
					cv::Scalar color,
					int thickness)
{
	int w = cvRound(blob.width);
	int h = cvRound(blob.height);

	int tl_x = cvRound(blob.x);
	int tl_y = cvRound(blob.y);

    int totallength = dashlength + linelength;
	int nCountX = w / totallength;
	int nCountY = h / totallength;

	cv::Point start, end;

	start.y=tl_y;
	start.x=tl_x;

	end.x = tl_x;
	end.y = tl_y;

	for (int i = 0; i < nCountX; i++) {
		end.x = tl_x + (i + 1) * totallength - dashlength;
		end.y = tl_y;
		start.x = tl_x + i * totallength;
		start.y = tl_y;
		cv::line(img, start, end, color, thickness);   
	}
	
	for (int i = 0;i < nCountX; i++) {  
		start.x = tl_x + i * totallength;
		start.y = tl_y + h;
		end.x = tl_x + (i + 1) * totallength - dashlength;
		end.y = tl_y + h;
		cv::line(img, start, end, color, thickness);     
	}

	for (int i = 0;i < nCountY; i++) {  
		start.x = tl_x;
		start.y = tl_y + i * totallength;
		end.y = tl_y + (i + 1) * totallength - dashlength;
		end.x = tl_x;
		cv::line(img, start, end, color, thickness);     
	}

	for (int i = 0;i < nCountY; i++) {  
		start.x = tl_x + w;
		start.y = tl_y + i * totallength;
		end.y = tl_y + (i + 1) * totallength - dashlength;
		end.x = tl_x + w;
		cv::line(img, start, end, color, thickness);     
	}
}