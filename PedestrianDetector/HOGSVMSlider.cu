#include "HOGSVMSlider.h"
#include "HOGUtils.h"

texture<float, 1, cudaReadModeElementType> texSVM;
cudaArray *svmArray = 0;

cudaChannelFormatDesc channelDescSVM;

extern int scaleCount;
extern int hNumberOfWindowsX, hNumberOfWindowsY;
extern int hNumberOfBlockPerWindowX, hNumberOfBlockPerWindowY;
extern int rNumberOfWindowsX, rNumberOfWindowsY;

extern __shared__ float1 allSharedF1[];

float svmBias;

__host__ void InitSVM(float _svmBias, float* svmWeights, int svmWeightsCount)
{
	channelDescSVM = cudaCreateChannelDesc<float>();
	checkCudaErrors(cudaMallocArray(&svmArray, &channelDescSVM, svmWeightsCount, 1));
	checkCudaErrors(cudaMemcpyToArray(svmArray, 0, 0, svmWeights, svmWeightsCount * sizeof(float), cudaMemcpyHostToDevice));
	svmBias = _svmBias;
}

__host__ void CloseSVM()
{
	checkCudaErrors(cudaFreeArray(svmArray));
}

__global__ void linearSVMEvaluation(float1* svmScores, float svmBias,
									float1* blockHistograms, int noHistogramBins,
									int windowSizeX, int windowSizeY, int hogBlockCountX, int hogBlockCountY,
									int cellSizeX, int cellSizeY,
									int numberOfBlockPerWindowX, int numberOfBlockPerWindowY,
									int blockSizeX, int blockSizeY,
									int alignedBlockDimX,
									int scaleId, int scaleCount,
									int hNumberOfWindowsX, int hNumberOfWindowsY,
									int width, int height)
{
	int i;
	int texPos;
	float1 localValue;
	float texValue;

	float1* smem = (float1*) allSharedF1;

	int gmemPosWindow, gmemPosInWindow, gmemPosInWindowDown, smemLocalPos, smemTargetPos;
	int gmemStride = hogBlockCountX * noHistogramBins * blockSizeX;
	
	// printf("blockHistograms %p\n", blockHistograms);
	
	// printf("gmemStride %d\n", gmemStride); // = 54

	gmemPosWindow = blockIdx.x * noHistogramBins * blockSizeX + blockIdx.y * blockSizeY * gmemStride;
	
	// if (blockIdx.x == 1 && blockIdx.y == 1)
	// printf("blockIdx.x %d, blockIdx.y %d\n", blockIdx.x, blockIdx.y);
	
	// printf("gmemPosWindow %d\n", gmemPosWindow); // = 0
	
	gmemPosInWindow = gmemPosWindow + threadIdx.x;
	
	// printf("threadIdx.x %d\n", threadIdx.x);
	
	// printf("gmemPosInWindow %d\n", gmemPosInWindow); // = 0 ~ 53
	
	smemLocalPos = threadIdx.x;

	int val1 = (blockSizeY * blockSizeX * noHistogramBins) * numberOfBlockPerWindowY;
	int val2 = blockSizeX * noHistogramBins; // 18
	localValue.x = 0;

	if (blockIdx.x == 10 && blockIdx.y == 8)
	{
		int asasasa;
		asasasa = 0;
		asasasa++;
	}

	for (i = 0; i<blockSizeY * numberOfBlockPerWindowY; i++)
	{
		gmemPosInWindowDown = gmemPosInWindow + i * gmemStride;
		texPos = threadIdx.x % val2 + i * val2 + threadIdx.x / val2 * val1;
		
		// if (blockIdx.x == 1 && blockIdx.y == 1 && threadIdx.x == 53)
		// printf("threadIdx.x %d, row %d, texPos %d\n", threadIdx.x, i, texPos);
		
		texValue =  tex1D(texSVM, texPos);
		localValue.x += blockHistograms[gmemPosInWindowDown].x * texValue;
	}

	smem[smemLocalPos] = localValue;

	__syncthreads();

	for(unsigned int s = alignedBlockDimX >> 1; s>0; s>>=1)
	{
		// printf("s %u\n", s);
		if (threadIdx.x < s && (threadIdx.x + s) < blockDim.x)
		{
			smemTargetPos = threadIdx.x + s;
			smem[smemLocalPos].x += smem[smemTargetPos].x;
			// printf("%d %u %d\n", smemLocalPos, s, smemTargetPos);
		}

		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		smem[smemLocalPos].x -= svmBias;
		svmScores[blockIdx.x + blockIdx.y * hNumberOfWindowsX + scaleId * hNumberOfWindowsX * hNumberOfWindowsY] = smem[smemLocalPos];
	}

	if (blockIdx.x == 10 && blockIdx.y == 8)
	{
		int asasasa;
		asasasa = 0;
		asasasa++;
	}
}

__host__ void ResetSVMScores(float1* svmScores)
{
	checkCudaErrors(cudaMemset(svmScores, 0, sizeof(float) * scaleCount * hNumberOfWindowsX * hNumberOfWindowsY));
}

__host__ void LinearSVMEvaluation(float1* svmScores, float1* blockHistograms, int noHistogramBins,
								  int windowSizeX, int windowSizeY,
								  int cellSizeX, int cellSizeY, int blockSizeX, int blockSizeY,
								  int hogBlockCountX, int hogBlockCountY,
								  int scaleId, int width, int height)
{
	rNumberOfWindowsX = (width-windowSizeX)/cellSizeX + 1;
	rNumberOfWindowsY = (height-windowSizeY)/cellSizeY + 1;

	dim3 threadCount = dim3(noHistogramBins * blockSizeX * hNumberOfBlockPerWindowX);
	dim3 blockCount = dim3(rNumberOfWindowsX, rNumberOfWindowsY);

	int alignedBlockDimX = iClosestPowerOfTwo(noHistogramBins * blockSizeX * hNumberOfBlockPerWindowX);

	checkCudaErrors(cudaBindTextureToArray(texSVM, svmArray, channelDescSVM));

	linearSVMEvaluation<<<blockCount, threadCount, noHistogramBins * blockSizeX * hNumberOfBlockPerWindowX * sizeof(float1)>>>
		(svmScores, svmBias, blockHistograms, noHistogramBins,
		windowSizeX, windowSizeY, hogBlockCountX, hogBlockCountY, cellSizeX, cellSizeY,
		hNumberOfBlockPerWindowX, hNumberOfBlockPerWindowY,
		blockSizeX, blockSizeY, alignedBlockDimX, scaleId, scaleCount,
		hNumberOfWindowsX, hNumberOfWindowsY, width, height);

	checkCudaErrors(cudaUnbindTexture(texSVM));
}
