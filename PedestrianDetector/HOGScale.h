#ifndef __HOG_SCALE__
#define __HOG_SCALE__

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef _WIN32
#  define WINDOWS_LEAN_AND_MEAN
#  include <windows.h>
#endif

#include <cuda_gl_interop.h>
// #include <cutil_inline.h>
#include <helper_cuda.h>
#include <cuda.h>

#include "HOGDefines.h"

__host__ void InitScale(int hPaddedWidth, int hPaddedHeight);
__host__ void CloseScale();

__host__ void DownscaleImage(int startScaleId, int endScaleId, int scaleId, float scale, 
							 bool useGrayscale, float4* paddedRegisteredImage,
							 float1* resizedPaddedImageF1, float4* resizedPaddedImageF4);
__host__ void DownscaleGrayImage(int startScaleId, int endScaleId, int scaleId, float scale, 
							 bool useGrayscale, float1* paddedRegisteredGrayImage,
							 float1* resizedPaddedImageF1);						 
__global__ void resizeFastBicubic1(float1 *outputFloat, float4* paddedRegisteredImage, int width, int height, float scale);
__global__ void resizeGrayFastBicubic1(float1 *outputFloat, float1* paddedRegisteredGrayImage, int width, int height, float scale);
__global__ void resizeFastBicubic4(float4 *outputFloat, float4* paddedRegisteredImage, int width, int height, float scale);

//__device__ float4 tex2DFastBicubic(const texture<uchar4, 2, cudaReadModeElementType> texref, float x, float y, float scale);
//
//__device__ float w0(float a);
//__device__ float w1(float a);
//__device__ float w2(float a);
//__device__ float w3(float a);
//
//__device__ float g0(float a);
//__device__ float g1(float a);
//
//__device__ float h0(float a);
//__device__ float h1(float a);

#endif
