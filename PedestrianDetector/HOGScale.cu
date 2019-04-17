#include "HOGScale.h"
#include "HOGUtils.h"

extern int rPaddedHeight;
extern int rPaddedWidth;
extern int hPaddedHeight;
extern int hPaddedWidth;
cudaArray *imageArray = 0;
texture<float4, 2, cudaReadModeElementType> tex;
texture<float1, 2, cudaReadModeElementType> grayTex;
cudaChannelFormatDesc channelDescDownscale;
cudaChannelFormatDesc grayChannelDescDownscale;

bool isAlocated;

// w0, w1, w2, and w3 are the four cubic B-spline basis functions
__device__ float w0(float a) { return (1.0f/6.0f)*(a*(a*(-a + 3.0f) - 3.0f) + 1.0f); }
__device__ float w1(float a) { return (1.0f/6.0f)*(a*a*(3.0f*a - 6.0f) + 4.0f); }
__device__ float w2(float a) { return (1.0f/6.0f)*(a*(a*(-3.0f*a + 3.0f) + 3.0f) + 1.0f); }
__device__ float w3(float a) { return (1.0f/6.0f)*(a*a*a); }

// g0 and g1 are the two amplitude functions
__device__ float g0(float a) { return w0(a) + w1(a); }
__device__ float g1(float a) { return w2(a) + w3(a); }

// h0 and h1 are the two offset functions
__device__ float h0(float a) { return -1.0f + w1(a) / (w0(a) + w1(a)) + 0.5f; }
__device__ float h1(float a) { return 1.0f + w3(a) / (w2(a) + w3(a)) + 0.5f; }

__host__ void InitScale(int hPaddedWidth, int hPaddedHeight)
{
	channelDescDownscale = cudaCreateChannelDesc<float4>();
	grayChannelDescDownscale = cudaCreateChannelDesc<float1>();
	tex.filterMode = cudaFilterModeLinear;
	tex.normalized = false;
	grayTex.filterMode = cudaFilterModeLinear;
	grayTex.normalized = false;
	isAlocated = false;
}

__host__ void CloseScale()
{
	//if (isAlocated) checkCudaErrors(cudaFreeArray(imageArray));
}

__host__ void DownscaleImage(int startScaleId, int endScaleId, int scaleId, float scale, 
							 bool useGrayscale, float4* paddedRegisteredImage,
							 float1* resizedPaddedImageF1, float4* resizedPaddedImageF4)
{
	dim3 hThreadSize, hBlockSize;

	hThreadSize = dim3(THREAD_SIZE_W, THREAD_SIZE_H);

	rPaddedWidth = iDivUpF(hPaddedWidth, scale);
	rPaddedHeight = iDivUpF(hPaddedHeight, scale);

	hBlockSize = dim3(iDivUp(rPaddedWidth, hThreadSize.x), iDivUp(rPaddedHeight, hThreadSize.y));

	if (scaleId == startScaleId)
	{
		if (isAlocated)
			checkCudaErrors(cudaFreeArray(imageArray));
		checkCudaErrors(cudaMallocArray(&imageArray, &channelDescDownscale, hPaddedWidth, hPaddedHeight) );
		checkCudaErrors(cudaMemcpyToArray(imageArray, 0, 0, paddedRegisteredImage, sizeof(float4) * hPaddedWidth * hPaddedHeight, cudaMemcpyDeviceToDevice));
		isAlocated = true;
	}

	checkCudaErrors(cudaBindTextureToArray(tex, imageArray, channelDescDownscale));

	if (useGrayscale)
	{
		checkCudaErrors(cudaMemset(resizedPaddedImageF1, 0, hPaddedWidth * hPaddedHeight * sizeof(float1)));
		resizeFastBicubic1<<<hBlockSize, hThreadSize>>>(resizedPaddedImageF1, paddedRegisteredImage, rPaddedWidth, rPaddedHeight, scale);
	}
	else
	{
		checkCudaErrors(cudaMemset(resizedPaddedImageF4, 0, hPaddedWidth * hPaddedHeight * sizeof(float4)));
		resizeFastBicubic4<<<hBlockSize, hThreadSize>>>(resizedPaddedImageF4, paddedRegisteredImage, rPaddedWidth, rPaddedHeight, scale);
	}

	checkCudaErrors(cudaUnbindTexture(tex));

	if (scaleId == endScaleId)
	{
		checkCudaErrors(cudaFreeArray(imageArray));
		isAlocated = false;
	}
}

__host__ void DownscaleGrayImage(int startScaleId, int endScaleId, int scaleId, float scale, 
							 bool useGrayscale, float1* paddedRegisteredGrayImage,
							 float1* resizedPaddedImageF1)
{
	dim3 hThreadSize, hBlockSize;

	hThreadSize = dim3(THREAD_SIZE_W, THREAD_SIZE_H);

	rPaddedWidth = iDivUpF(hPaddedWidth, scale);
	rPaddedHeight = iDivUpF(hPaddedHeight, scale);

	hBlockSize = dim3(iDivUp(rPaddedWidth, hThreadSize.x), iDivUp(rPaddedHeight, hThreadSize.y));

	if (scaleId == startScaleId)
	{
		if (isAlocated)
			checkCudaErrors(cudaFreeArray(imageArray));
		checkCudaErrors(cudaMallocArray(&imageArray, &grayChannelDescDownscale, hPaddedWidth, hPaddedHeight) );
		checkCudaErrors(cudaMemcpyToArray(imageArray, 0, 0, paddedRegisteredGrayImage, sizeof(float1) * hPaddedWidth * hPaddedHeight, cudaMemcpyDeviceToDevice));
		isAlocated = true;
	}

	checkCudaErrors(cudaBindTextureToArray(grayTex, imageArray, grayChannelDescDownscale));

	checkCudaErrors(cudaMemset(resizedPaddedImageF1, 0, hPaddedWidth * hPaddedHeight * sizeof(float1)));
	resizeGrayFastBicubic1<<<hBlockSize, hThreadSize>>>(resizedPaddedImageF1, paddedRegisteredGrayImage, rPaddedWidth, rPaddedHeight, scale);

	checkCudaErrors(cudaUnbindTexture(grayTex));

	if (scaleId == endScaleId)
	{
		checkCudaErrors(cudaFreeArray(imageArray));
		isAlocated = false;
	}
}

__device__ float4 tex2DFastBicubic(const texture<float4, 2, cudaReadModeElementType> texref, float x, float y)
{
	float4 r;
	float4 val0, val1, val2, val3;

	x -= 0.5f;
	y -= 0.5f;
	float px = floor(x);
	float py = floor(y);
	float fx = x - px;
	float fy = y - py;

	float g0x = g0(fx);
	float g1x = g1(fx);
	float h0x = h0(fx);
	float h1x = h1(fx);
	float h0y = h0(fy);
	float h1y = h1(fy);

	val0 = tex2D(texref, px + h0x, py + h0y);
	val1 = tex2D(texref, px + h1x, py + h0y);
	val2 = tex2D(texref, px + h0x, py + h1y);
	val3 = tex2D(texref, px + h1x, py + h1y);

	r.x = (g0(fy) * (g0x * val0.x + g1x * val1.x) + g1(fy) * (g0x * val2.x + g1x * val3.x));
	r.y = (g0(fy) * (g0x * val0.y + g1x * val1.y) + g1(fy) * (g0x * val2.y + g1x * val3.y));
	r.z = (g0(fy) * (g0x * val0.z + g1x * val1.z) + g1(fy) * (g0x * val2.z + g1x * val3.z));
	r.w = (g0(fy) * (g0x * val0.w + g1x * val1.w) + g1(fy) * (g0x * val2.w + g1x * val3.w));

	return r;
}

__global__ void resizeFastBicubic4(float4 *outputFloat, float4* paddedRegisteredImage, int width, int height, float scale)
{
	int x = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
	int y = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;
	int i = __umul24(y, width) + x;

	float u = x*scale;
	float v = y*scale;

	if (x < width && y < height)
	{
		float4 cF;

		if (scale == 1.0f)
		{
			cF = paddedRegisteredImage[x + y * width];
			cF.w = 0;
		}
		else
		{
			cF = tex2D(tex, u, v);
			cF.w = 0;
		}

		cF.x = sqrtf(cF.x); cF.y = sqrtf(cF.y); cF.z = sqrtf(cF.z); cF.w = 0;
		outputFloat[i] = cF;
	}
}

__global__ void resizeFastBicubic1(float1 *outputFloat, float4* paddedRegisteredImage, int width, int height, float scale)
{
	int x = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
	int y = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;
	int i = __umul24(y, width) + x;

	float u = x*scale;
	float v = y*scale;

	if (x < width && y < height)
	{
		float4 cF;

		if (scale == 1.0f)
		{
			cF = paddedRegisteredImage[x + y * width];
			cF.w = 0;
		}
		else
		{
			cF = tex2D(tex, u, v);
			cF.w = 0;
		}

		outputFloat[i].x = sqrtf(0.2989f * cF.x + 0.5870f * cF.y + 0.1140f * cF.z);
	}
}

__global__ void resizeGrayFastBicubic1(float1 *outputFloat, float1* paddedRegisteredGrayImage, int width, int height, float scale)
{
	int x = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
	int y = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;
	int i = __umul24(y, width) + x;

	float u = x*scale;
	float v = y*scale;

	if (x < width && y < height)
	{
		float1 cF;

		if (scale == 1.0f)
		{
			cF = paddedRegisteredGrayImage[x + y * width];
		}
		else
		{
			cF = tex2D(grayTex, u, v);
		}

		outputFloat[i].x = cF.x;
	}
}
