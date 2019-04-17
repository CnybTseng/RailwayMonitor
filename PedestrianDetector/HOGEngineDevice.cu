#include "HOGEngineDevice.h"
#include "HOGUtils.h"
#include "HOGConvolution.h"
#include "HOGHistogram.h"
#include "HOGSVMSlider.h"
#include "HOGScale.h"
#include "HOGPadding.h"

int hWidth, hHeight;
int hWidthROI, hHeightROI;
int hPaddedWidth, hPaddedHeight;
int rPaddedWidth, rPaddedHeight;

int minX, minY, maxX, maxY;

int hNoHistogramBins, rNoHistogramBins;

int hPaddingSizeX, hPaddingSizeY;
int hCellSizeX, hCellSizeY, hBlockSizeX, hBlockSizeY, hWindowSizeX, hWindowSizeY;
int hNoOfCellsX, hNoOfCellsY, hNoOfBlocksX, hNoOfBlocksY;
int rNoOfCellsX, rNoOfCellsY, rNoOfBlocksX, rNoOfBlocksY;

int hNumberOfBlockPerWindowX, hNumberOfBlockPerWindowY;
int hNumberOfWindowsX, hNumberOfWindowsY;
int rNumberOfWindowsX, rNumberOfWindowsY;

float4 *paddedRegisteredImage;
float1 *paddedRegisteredGrayImage;

float1 *resizedPaddedImageF1;
float4 *resizedPaddedImageF4;

float2 *colorGradientsF2;

float1 *blockHistograms;
float1 *cellHistograms;

float1 *svmScores;

bool hUseGrayscale;

// uchar1* outputTest1;
uchar4* outputTest4;

float* hResult;

float scaleRatio;
float startScale;
float endScale;
int scaleCount;

int avSizeX, avSizeY, marginX, marginY;

extern uchar4* paddedRegisteredImageU4;

float1 *deviceImage;

__host__ void InitHOG(int width, int height,
					  int _avSizeX, int _avSizeY,
					  int _marginX, int _marginY,
					  int cellSizeX, int cellSizeY,
					  int blockSizeX, int blockSizeY,
					  int windowSizeX, int windowSizeY,
					  int noOfHistogramBins, float wtscale,
					  float svmBias, float* svmWeights, int svmWeightsCount,
					  bool useGrayscale)
{
	cudaSetDevice( gpuGetMaxGflopsDeviceId() );

	int i;
	int toaddxx = 0, toaddxy = 0, toaddyx = 0, toaddyy = 0;

	hWidth = width; hHeight = height;
	avSizeX = _avSizeX; avSizeY = _avSizeY; marginX = _marginX; marginY = _marginY;

	if (avSizeX) { toaddxx = hWidth * marginX / avSizeX; toaddxy = hHeight * marginY / avSizeX; }
	if (avSizeY) { toaddyx = hWidth * marginX / avSizeY; toaddyy = hHeight * marginY / avSizeY; }

	hPaddingSizeX = max(toaddxx, toaddyx); hPaddingSizeY = max(toaddxy, toaddyy);

	hPaddedWidth = hWidth + hPaddingSizeX*2;
	hPaddedHeight = hHeight + hPaddingSizeY*2;

	hUseGrayscale = useGrayscale;

	hNoHistogramBins = noOfHistogramBins;
	hCellSizeX = cellSizeX; hCellSizeY = cellSizeY; hBlockSizeX = blockSizeX; hBlockSizeY = blockSizeY;
	hWindowSizeX = windowSizeX; hWindowSizeY = windowSizeY;

	hNoOfCellsX = hPaddedWidth / cellSizeX;
	hNoOfCellsY = hPaddedHeight / cellSizeY;

	hNoOfBlocksX = hNoOfCellsX - blockSizeX + 1;
	hNoOfBlocksY = hNoOfCellsY - blockSizeY + 1;

	hNumberOfBlockPerWindowX = (windowSizeX - cellSizeX * blockSizeX) / cellSizeX + 1;
	hNumberOfBlockPerWindowY = (windowSizeY - cellSizeY * blockSizeY) / cellSizeY + 1;

	hNumberOfWindowsX = 0;
	for (i=0; i<hNumberOfBlockPerWindowX; i++) hNumberOfWindowsX += (hNoOfBlocksX-i)/hNumberOfBlockPerWindowX;

	hNumberOfWindowsY = 0;
	for (i=0; i<hNumberOfBlockPerWindowY; i++) hNumberOfWindowsY += (hNoOfBlocksY-i)/hNumberOfBlockPerWindowY;

	scaleRatio = 1.05f;
	startScale = 1.0f;
	endScale = min(hPaddedWidth / (float) hWindowSizeX, hPaddedHeight / (float) hWindowSizeY);
	scaleCount = (int)floor(logf(endScale/startScale)/logf(scaleRatio)) + 1;

	checkCudaErrors(cudaMalloc((void**) &paddedRegisteredImage, sizeof(float4) * hPaddedWidth * hPaddedHeight));
	
	checkCudaErrors(cudaMalloc((void**) &paddedRegisteredGrayImage, sizeof(float1) * hPaddedWidth * hPaddedHeight));

	if (useGrayscale)
		checkCudaErrors(cudaMalloc((void**) &resizedPaddedImageF1, sizeof(float1) * hPaddedWidth * hPaddedHeight));
	else
		checkCudaErrors(cudaMalloc((void**) &resizedPaddedImageF4, sizeof(float4) * hPaddedWidth * hPaddedHeight));

	checkCudaErrors(cudaMalloc((void**) &colorGradientsF2, sizeof(float2) * hPaddedWidth * hPaddedHeight));
	checkCudaErrors(cudaMalloc((void**) &blockHistograms, sizeof(float1) * hNoOfBlocksX * hNoOfBlocksY * cellSizeX * cellSizeY * hNoHistogramBins));
	checkCudaErrors(cudaMalloc((void**) &cellHistograms, sizeof(float1) * hNoOfCellsX * hNoOfCellsY * hNoHistogramBins));

	checkCudaErrors(cudaMalloc((void**) &svmScores, sizeof(float1) * hNumberOfWindowsX * hNumberOfWindowsY * scaleCount));

	InitConvolution(hPaddedWidth, hPaddedHeight, useGrayscale);
	InitHistograms(cellSizeX, cellSizeY, blockSizeX, blockSizeY, noOfHistogramBins, wtscale);
	InitSVM(svmBias, svmWeights, svmWeightsCount);
	InitScale(hPaddedWidth, hPaddedHeight);
	InitPadding(hPaddedWidth, hPaddedHeight);

	rPaddedWidth = hPaddedWidth;
	rPaddedHeight = hPaddedHeight;

	// if (useGrayscale)
		// checkCudaErrors(cudaMalloc((void**) &outputTest1, sizeof(uchar1) * hPaddedWidth * hPaddedHeight));
	// else
		checkCudaErrors(cudaMalloc((void**) &outputTest4, sizeof(uchar4) * hPaddedWidth * hPaddedHeight));

	checkCudaErrors(cudaMallocHost((void**)&hResult, sizeof(float) * hNumberOfWindowsX * hNumberOfWindowsY * scaleCount));
}

__host__ void CloseHOG()
{
	checkCudaErrors(cudaFree(paddedRegisteredImage));
	
	checkCudaErrors(cudaFree(paddedRegisteredGrayImage));

	if (hUseGrayscale)
		checkCudaErrors(cudaFree(resizedPaddedImageF1));
	else
		checkCudaErrors(cudaFree(resizedPaddedImageF4));

	checkCudaErrors(cudaFree(colorGradientsF2));
	checkCudaErrors(cudaFree(blockHistograms));
	checkCudaErrors(cudaFree(cellHistograms));

	checkCudaErrors(cudaFree(svmScores));

	CloseConvolution();
	CloseHistogram();
	CloseSVM();
	CloseScale();
	ClosePadding();

	// if (hUseGrayscale)
		// checkCudaErrors(cudaFree(outputTest1));
	// else
		checkCudaErrors(cudaFree(outputTest4));

	checkCudaErrors(cudaFreeHost(hResult));

	cudaThreadExit();
}

__host__ void BeginHOGProcessing(unsigned char* hostImage, int minx, int miny, int maxx, int maxy, float minScale, float maxScale)
{
	int i;
	minX = minx; minY = miny; maxX = maxx; maxY = maxy;
	
	if (hUseGrayscale) {
		PadHostGrayImage((uchar4*)hostImage, paddedRegisteredGrayImage, minX, minY, maxX, maxY);
	} else {
		PadHostImage((uchar4*)hostImage, paddedRegisteredImage, minX, minY, maxX, maxY);
	}
		
	rPaddedWidth = hPaddedWidth; rPaddedHeight = hPaddedHeight;
	scaleRatio = 1.05f;
	startScale = (minScale < 0.0f) ? 1.0f : minScale;
	endScale = (maxScale < 0.0f) ? min(hPaddedWidth / (float) hWindowSizeX, hPaddedHeight / (float) hWindowSizeY) : maxScale;
	scaleCount = (int)floor(logf(endScale/startScale)/logf(scaleRatio)) + 1;

	float currentScale = startScale;

	ResetSVMScores(svmScores);

	for (i=0; i<scaleCount; i++)
	{
		if (hUseGrayscale) {
			DownscaleGrayImage(0, scaleCount, i, currentScale, hUseGrayscale, paddedRegisteredGrayImage, resizedPaddedImageF1);
		} else {
			DownscaleImage(0, scaleCount, i, currentScale, hUseGrayscale, paddedRegisteredImage, resizedPaddedImageF1, resizedPaddedImageF4);
		}
		
		SetConvolutionSize(rPaddedWidth, rPaddedHeight);

		if (hUseGrayscale) ComputeColorGradients1to2(resizedPaddedImageF1, colorGradientsF2);
		else ComputeColorGradients4to2(resizedPaddedImageF4, colorGradientsF2);

		ComputeBlockHistogramsWithGauss(colorGradientsF2, blockHistograms, hNoHistogramBins,
			hCellSizeX, hCellSizeY, hBlockSizeX, hBlockSizeY, hWindowSizeX, hWindowSizeY,  rPaddedWidth, rPaddedHeight);

		NormalizeBlockHistograms(blockHistograms, hNoHistogramBins, hCellSizeX, hCellSizeY, hBlockSizeX, hBlockSizeY, rPaddedWidth, rPaddedHeight);

		LinearSVMEvaluation(svmScores, blockHistograms, hNoHistogramBins, hWindowSizeX, hWindowSizeY, hCellSizeX, hCellSizeY,
			hBlockSizeX, hBlockSizeY, rNoOfBlocksX, rNoOfBlocksY, i, rPaddedWidth, rPaddedHeight);

		currentScale *= scaleRatio;
	}
}

__host__ float* EndHOGProcessing()
{
	cudaThreadSynchronize();
	checkCudaErrors(cudaMemcpy(hResult, svmScores, sizeof(float) * scaleCount * hNumberOfWindowsX * hNumberOfWindowsY, cudaMemcpyDeviceToHost));
	return hResult;
}

__host__ void CalculateHOGDescriptor(float *hostImage, int width, int height, int noOfHistogramBins,
                                     int windowSizeX, int windowSizeY, int cellSizeX, int cellSizeY,
                                     int blockSizeX, int blockSizeY, float *hostDesc, float *gradient)
{
	cudaSetDevice(gpuGetMaxGflopsDeviceId());
	
	int noOfBlocksX;
	int noOfBlocksY;
	float1 *deviceImage;
	float2 *colorGradientsF2;
	float1 *blockHistograms;
	int y;
	
	noOfBlocksX = width / cellSizeX - blockSizeX + 1;
	noOfBlocksY = height / cellSizeY - blockSizeY + 1;

	checkCudaErrors(cudaMalloc((void**)&deviceImage, sizeof(float1) * width * height));

	checkCudaErrors(cudaMalloc((void**)&colorGradientsF2, sizeof(float2) * width * height));

	checkCudaErrors(cudaMalloc((void**)&blockHistograms, sizeof(float1) * noOfBlocksX * noOfBlocksY * blockSizeX * blockSizeY * noOfHistogramBins));
	
	checkCudaErrors(cudaMalloc((void**) &outputTest4, sizeof(uchar4) * width * height));
	
	InitConvolution(width, height, true);
	InitHistograms(cellSizeX, cellSizeY, blockSizeX, blockSizeY, noOfHistogramBins, 2.0);
	
	checkCudaErrors(cudaMemcpy2D(deviceImage, sizeof(float1) * width, hostImage, sizeof(float1) * width, sizeof(float1) * width, height, cudaMemcpyHostToDevice));

	SetConvolutionSize(width, height);
	
	ComputeColorGradients1to2(deviceImage, colorGradientsF2);

	checkCudaErrors(cudaMemcpy(colorGradientsF2, colorGradientsF2 + width, sizeof(float2) * width, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(colorGradientsF2 + width * (height - 1), colorGradientsF2 + width * (height - 2), sizeof(float2) * width, cudaMemcpyDeviceToDevice));
	
	for (y = 0; y < height; y++) {
		checkCudaErrors(cudaMemcpy(colorGradientsF2 + y * width, colorGradientsF2 + y * width + 1, sizeof(float2), cudaMemcpyDeviceToDevice));
		checkCudaErrors(cudaMemcpy(colorGradientsF2 + y * width + width - 1, colorGradientsF2 + y * width + width - 2, sizeof(float2), cudaMemcpyDeviceToDevice));
	}
	
	ComputeBlockHistogramsWithGauss(colorGradientsF2, blockHistograms, noOfHistogramBins, cellSizeX, cellSizeY, blockSizeX, blockSizeY, windowSizeX, windowSizeY, width, height);

	NormalizeBlockHistograms(blockHistograms, noOfHistogramBins, cellSizeX, cellSizeY, blockSizeX, blockSizeY, width, height);

	Float2ToUchar4(colorGradientsF2, outputTest4, width, height, 0);
	checkCudaErrors(cudaMemcpy2D(gradient, width * sizeof(float1), outputTest4, width * sizeof(float1), width * sizeof(float1), height, cudaMemcpyDeviceToHost));
			
	checkCudaErrors(cudaMemcpy(hostDesc, blockHistograms, sizeof(float1) * noOfBlocksX * noOfBlocksY * blockSizeX * blockSizeY * noOfHistogramBins, cudaMemcpyDeviceToHost));
	
	checkCudaErrors(cudaFree(deviceImage));
	checkCudaErrors(cudaFree(colorGradientsF2));
	checkCudaErrors(cudaFree(blockHistograms));
	checkCudaErrors(cudaFree(outputTest4));
	
	CloseConvolution();
	CloseHistogram();
	
	cudaThreadExit();

	cudaThreadSynchronize();
}

__host__ void InitHOGDescriptorCalculator(int width, int height, int noOfHistogramBins,
                                          int windowSizeX, int windowSizeY, int cellSizeX, int cellSizeY,
                                          int blockSizeX, int blockSizeY, float wtscale)
{
	hWidth = width;
	hHeight = height;
	hNoHistogramBins = noOfHistogramBins;
	hWindowSizeX = windowSizeX;
	hWindowSizeY = windowSizeY;
	hCellSizeX = cellSizeX;
	hCellSizeY = cellSizeY;
	hBlockSizeX = blockSizeX;
	hBlockSizeY = blockSizeY;
	
	hNoOfBlocksX = width / cellSizeX - blockSizeX + 1;
	hNoOfBlocksY = height / cellSizeY - blockSizeY + 1;
	
	checkCudaErrors(cudaMalloc((void**)&deviceImage, sizeof(float1) * width * height));

	checkCudaErrors(cudaMalloc((void**)&colorGradientsF2, sizeof(float2) * width * height));

	checkCudaErrors(cudaMalloc((void**)&blockHistograms, sizeof(float1) * hNoOfBlocksX * hNoOfBlocksY * blockSizeX * blockSizeY * noOfHistogramBins));
	
	checkCudaErrors(cudaMalloc((void**) &outputTest4, sizeof(uchar4) * width * height));
	
	InitConvolution(width, height, true);
	InitHistograms(cellSizeX, cellSizeY, blockSizeX, blockSizeY, noOfHistogramBins, wtscale);
}

__host__ void HOGDescriptorCalculator(float *hostImage, float *hostDesc)
{
	int y;
	checkCudaErrors(cudaMemcpy2D(deviceImage, sizeof(float1) * hWidth, hostImage, sizeof(float1) * hWidth, sizeof(float1) * hWidth, hHeight, cudaMemcpyHostToDevice));
	
	SetConvolutionSize(hWidth, hHeight);
	
	ComputeColorGradients1to2(deviceImage, colorGradientsF2);

	checkCudaErrors(cudaMemcpy(colorGradientsF2, colorGradientsF2 + hWidth, sizeof(float2) * hWidth, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(colorGradientsF2 + hWidth * (hHeight - 1), colorGradientsF2 + hWidth * (hHeight - 2), sizeof(float2) * hWidth, cudaMemcpyDeviceToDevice));
	
	for (y = 0; y < hHeight; y++) {
		checkCudaErrors(cudaMemcpy(colorGradientsF2 + y * hWidth, colorGradientsF2 + y * hWidth + 1, sizeof(float2), cudaMemcpyDeviceToDevice));
		checkCudaErrors(cudaMemcpy(colorGradientsF2 + y * hWidth + hWidth - 1, colorGradientsF2 + y * hWidth + hWidth - 2, sizeof(float2), cudaMemcpyDeviceToDevice));
	}
	
	ComputeBlockHistogramsWithGauss(colorGradientsF2, blockHistograms, hNoHistogramBins, hCellSizeX, hCellSizeY, hBlockSizeX, hBlockSizeY, hWindowSizeX, hWindowSizeY, hWidth, hHeight);

	NormalizeBlockHistograms(blockHistograms, hNoHistogramBins, hCellSizeX, hCellSizeY, hBlockSizeX, hBlockSizeY, hWidth, hHeight);

	checkCudaErrors(cudaMemcpy(hostDesc, blockHistograms, sizeof(float1) * hNoOfBlocksX * hNoOfBlocksY * hBlockSizeX * hBlockSizeY * hNoHistogramBins, cudaMemcpyDeviceToHost));
}

__host__ void HOGDescriptorCalculatorOG(float *hostImage, float *hostDesc, float *gradient)
{
	HOGDescriptorCalculator(hostImage, hostDesc);
	
	Float2ToUchar4(colorGradientsF2, outputTest4, hWidth, hHeight, 0);
	checkCudaErrors(cudaMemcpy2D(gradient, hWidth * sizeof(float1), outputTest4, hWidth * sizeof(float1), hWidth * sizeof(float1), hHeight, cudaMemcpyDeviceToHost));
}

__host__ void FreeHOGDescriptorCalculator()
{
	checkCudaErrors(cudaFree(deviceImage));
	checkCudaErrors(cudaFree(colorGradientsF2));
	checkCudaErrors(cudaFree(blockHistograms));
	checkCudaErrors(cudaFree(outputTest4));
	
	CloseConvolution();
	CloseHistogram();
	
	cudaThreadExit();

	cudaThreadSynchronize();
}

__host__ void GetProcessedImage(unsigned char* hostImage, int imageType)
{
		switch (imageType)
		{
		case 0:
			Float4ToUchar4(resizedPaddedImageF4, outputTest4, rPaddedWidth, rPaddedHeight);
			break;
		case 1:
			Float2ToUchar4(colorGradientsF2, outputTest4, rPaddedWidth, rPaddedHeight, 0);
			break;
		case 2:
			Float2ToUchar4(colorGradientsF2, outputTest4, rPaddedWidth, rPaddedHeight, 1);
			break;
		case 3:
			checkCudaErrors(cudaMemcpy(hostImage, paddedRegisteredImageU4, sizeof(uchar4) * hPaddedWidth * hPaddedHeight, cudaMemcpyDeviceToHost));
			return;
		case 4:
			checkCudaErrors(cudaMemcpy2D(((uchar4*)hostImage) + minX + minY * hWidth, hWidth * sizeof(uchar4), 
				paddedRegisteredImageU4 + hPaddingSizeX + hPaddingSizeY * hPaddedWidth, hPaddedWidth * sizeof(uchar4),
				hWidthROI * sizeof(uchar4), hHeightROI, cudaMemcpyDeviceToHost));
			return;
		}

		checkCudaErrors(cudaMemcpy2D(hostImage, hPaddedWidth * sizeof(uchar4), outputTest4, rPaddedWidth * sizeof(uchar4),
			rPaddedWidth * sizeof(uchar4), rPaddedHeight, cudaMemcpyDeviceToHost));

	//checkCudaErrors(cudaMemcpy(hostImage, paddedRegisteredImage, sizeof(uchar4) * hPaddedWidth * hPaddedHeight, cudaMemcpyDeviceToHost));
}

__host__ void GetHOGDescriptor(float *hostDesc)
{
	checkCudaErrors(cudaMemcpy(hostDesc, blockHistograms, sizeof(float1) * hNoOfBlocksX * hNoOfBlocksY * hCellSizeX * hCellSizeY * hNoHistogramBins, cudaMemcpyDeviceToHost));
}

__host__ void GetHOGParameters(float *cStartScale, float *cEndScale, float *cScaleRatio, int *cScaleCount,
							   int *cPaddingSizeX, int *cPaddingSizeY, int *cPaddedWidth, int *cPaddedHeight,
							   int *cNoOfCellsX, int *cNoOfCellsY, int *cNoOfBlocksX, int *cNoOfBlocksY,
							   int *cNumberOfWindowsX, int *cNumberOfWindowsY,
							   int *cNumberOfBlockPerWindowX, int *cNumberOfBlockPerWindowY)
{
	*cStartScale = startScale;
	*cEndScale = endScale;
	*cScaleRatio = scaleRatio;
	*cScaleCount = scaleCount;
	*cPaddingSizeX = hPaddingSizeX;
	*cPaddingSizeY = hPaddingSizeY;
	*cPaddedWidth = hPaddedWidth;
	*cPaddedHeight = hPaddedHeight;
	*cNoOfCellsX = hNoOfCellsX;
	*cNoOfCellsY = hNoOfCellsY;
	*cNoOfBlocksX = hNoOfBlocksX;
	*cNoOfBlocksY = hNoOfBlocksY;
	*cNumberOfWindowsX = hNumberOfWindowsX;
	*cNumberOfWindowsY = hNumberOfWindowsY;
	*cNumberOfBlockPerWindowX = hNumberOfBlockPerWindowX;
	*cNumberOfBlockPerWindowY = hNumberOfBlockPerWindowY;
}

cudaArray *imageArray2 = 0;
texture<float4, 2, cudaReadModeElementType> tex2;
cudaChannelFormatDesc channelDescDownscale2;

__global__ void resizeFastBicubic3(float4 *outputFloat, float4* paddedRegisteredImage, int width, int height, float scale)
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
			cF = paddedRegisteredImage[x + y * width];
		else
			cF = tex2D(tex2, u, v);

		outputFloat[i] = cF;
	}
}

__host__ void DownscaleImage2(float scale, float4* paddedRegisteredImage,
							  float4* resizedPaddedImageF4, int width, int height,
							  int &rPaddedWidth, int &rPaddedHeight)
{
	dim3 hThreadSize, hBlockSize;

	hThreadSize = dim3(THREAD_SIZE_W, THREAD_SIZE_H);

	rPaddedWidth = iDivUpF(width, scale);
	rPaddedHeight = iDivUpF(height, scale);

	hBlockSize = dim3(iDivUp(rPaddedWidth, hThreadSize.x), iDivUp(rPaddedHeight, hThreadSize.y));

	checkCudaErrors(cudaMemcpyToArray(imageArray2, 0, 0, paddedRegisteredImage, sizeof(float4) * width * height, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaBindTextureToArray(tex2, imageArray2, channelDescDownscale2));

	checkCudaErrors(cudaMemset(resizedPaddedImageF4, 0, width * height * sizeof(float4)));
	resizeFastBicubic3<<<hBlockSize, hThreadSize>>>((float4*)resizedPaddedImageF4, (float4*)paddedRegisteredImage, rPaddedWidth, rPaddedHeight, scale);

	checkCudaErrors(cudaUnbindTexture(tex2));
}

__host__ float3* CUDAImageRescale(float3* src, int width, int height, int &rWidth, int &rHeight, float scale)
{
	int i, j, offsetC, offsetL;

	float4* srcH; float4* srcD;
	float4* dstD; float4* dstH;
	float3 val3; float4 val4;

	channelDescDownscale2 = cudaCreateChannelDesc<float4>();
	tex2.filterMode = cudaFilterModeLinear; tex2.normalized = false;

	cudaMalloc((void**)&srcD, sizeof(float4) * width * height);
	cudaMalloc((void**)&dstD, sizeof(float4) * width * height);
	cudaMallocHost((void**)&srcH, sizeof(float4) * width * height);
	cudaMallocHost((void**)&dstH, sizeof(float4) * width * height);
	checkCudaErrors(cudaMallocArray(&imageArray2, &channelDescDownscale2, width, height) );

	for (i=0; i<width; i++)
	{
		for (j=0; j<height; j++)
		{
			offsetC = j + i * height;
			offsetL = j * width + i;

			val3 = src[offsetC];

			srcH[offsetL].x = val3.x;
			srcH[offsetL].y = val3.y;
			srcH[offsetL].z = val3.z;
		}
	}
	cudaMemcpy(srcD, srcH, sizeof(float4) * width * height, cudaMemcpyHostToDevice);

	DownscaleImage2(scale, srcD, dstD, width, height, rWidth, rHeight);

	cudaMemcpy(dstH, dstD, sizeof(float4) * rWidth * rHeight, cudaMemcpyDeviceToHost);

	float3* dst = (float3*) malloc (rWidth * rHeight * sizeof(float3));
	for (i=0; i<rWidth; i++)
	{
		for (j=0; j<rHeight; j++)
		{
			offsetC = j + i * rHeight;
			offsetL = j * rWidth + i;

			val4 = dstH[offsetL];

			dst[offsetC].x = val4.x;
			dst[offsetC].y = val4.y;
			dst[offsetC].z = val4.z;
		}
	}

	checkCudaErrors(cudaFreeArray(imageArray2));
	cudaFree(srcD);
	cudaFree(dstD);
	cudaFreeHost(srcH);
	cudaFreeHost(dstH);

	return dst;
}
