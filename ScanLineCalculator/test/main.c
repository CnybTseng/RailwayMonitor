#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <windows.h>
#include <conio.h>

#include "scanlineparam.h"

int main()
{
	CalScanLineParam_initialize();
	
	float numinter = 20;
	float mininter = 4;
	float suminter = 400;
	float maxinter = 1;
	float scale = 1;
	
	LARGE_INTEGER freq;
	LARGE_INTEGER start = {0, 0};
	LARGE_INTEGER stop = {0, 0};
	QueryPerformanceFrequency(&freq); 
	
	QueryPerformanceCounter(&start);
	CalScanLineParam(numinter, mininter, suminter, &maxinter, &scale);
	QueryPerformanceCounter(&stop);
	printf("%lfs.\n", (double)(stop.QuadPart-start.QuadPart)/(double)freq.QuadPart);
	printf("maxinter=%f, scale=%f.\n", maxinter, scale);
	
	CalScanLineParam_terminate();
}