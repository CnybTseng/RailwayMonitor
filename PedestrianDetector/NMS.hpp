#ifndef _NMS_HPP_
#define _NMS_HPP_

#include <algorithm>
#include <vector>
#include <cstdint>

typedef struct {
	uint32_t lx;
	uint32_t ty;
	uint32_t width;
	uint32_t height;
	float score;
	float scale;
}DetectResult;

bool CompareDetection(DetectResult a,
                      DetectResult b)
{
	return (a.score > b.score);
}

void NonmaximumSuppression(std::vector<DetectResult> detect_result,
                           float overlap_thresh,
						   int window_width,
						   int window_height,
                           std::vector<DetectResult> &merged_result)
{
	if (0 == detect_result.size()) {
		return;
	}
	
	std::vector<DetectResult>::iterator it;	
	std::sort(detect_result.begin(), detect_result.end(), CompareDetection);
	
	while (detect_result.size() > 0) {
		it = detect_result.begin();
		merged_result.push_back(*it);
		DetectResult first = *it;
		it = detect_result.erase(it);

		for (it = detect_result.begin(); it != detect_result.end();) {
			uint32_t frx = first.lx + first.width;
			uint32_t fby = first.ty + first.height;
			uint32_t irx = it->lx + it->width;
			uint32_t iby = it->ty + it->height;
			
			int32_t olx = std::max(first.lx, it->lx);
			int32_t oty = std::max(first.ty, it->ty);
			
			int32_t orx = std::min(frx, irx);
			int32_t oby = std::min(fby, iby);
			
			int32_t w = orx - olx + 1;
			int32_t h = oby - oty + 1;
			if (w > 0 && h > 0) {	
				float fa = ((float)first.width) * (first.height);
				float ia = ((float)it->width) * (it->height);
				float oa = w * h / std::min(fa, ia);

				if (oa > overlap_thresh) {
					it = detect_result.erase(it);
				} else {
					it++;
				}
			} else {
				it++;
			}
		}
	}
}

#endif