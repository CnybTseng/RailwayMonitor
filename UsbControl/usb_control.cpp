#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include "usb_control.h"

HANDLE open_usb_device(int port, int baud_rate, int byte_size, int parity, int stop_bits)
{
#ifdef UNICODE
	LPTSTR filename = new TCHAR[128];
	wsprintf(filename, L"\\\\.\\COM%d", port);
	HANDLE device = CreateFile(filename, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	delete [] filename;
#else
	char filename[128];
	sprintf(filename, "\\\\.\\COM%d", port);
	HANDLE device = CreateFile(filename, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
#endif
	if (device == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "open COM%d fail: %d\n", port, GetLastError());
		return INVALID_HANDLE_VALUE;
	}
	
	DCB dcb = {0};
	dcb.DCBlength = sizeof(DCB);
	if (!GetCommState(device, &dcb)) {
		fprintf(stderr, "GetCommState fail: %d\n", GetLastError());
		CloseHandle(device);
		return INVALID_HANDLE_VALUE;
	}

	// printf("original sets:\n");
	// printf("baud rate:%d\nbyte size:%d\nparity:%d\nstop bits:%d\n", dcb.BaudRate, dcb.ByteSize, dcb.Parity, dcb.StopBits);
	
	dcb.BaudRate = baud_rate;
	dcb.ByteSize = byte_size;
	dcb.Parity = parity;
	dcb.StopBits = stop_bits;
	
	if (!SetCommState(device, &dcb)) {
		fprintf(stderr, "SetCommState fail: %d\n", GetLastError());
		CloseHandle(device);
		return INVALID_HANDLE_VALUE;
	}
	
	if (!GetCommState(device, &dcb)) {
		fprintf(stderr, "GetCommState: %d\n", GetLastError());
		CloseHandle(device);
		return INVALID_HANDLE_VALUE;
	}

	// printf("newest sets:\n");
	// printf("baud rate:%d\nbyte size:%d\nparity:%d\nstop bits:%d\n", dcb.BaudRate, dcb.ByteSize, dcb.Parity, dcb.StopBits);
	
	if (!PurgeComm(device, PURGE_RXCLEAR)) {
		fprintf(stderr, "PurgeComm fail: %d\n", GetLastError());
		CloseHandle(device);
		return INVALID_HANDLE_VALUE;
	}
	
	SetupComm(device, 1024, 1024);
	
	COMMTIMEOUTS timeout;
	timeout.ReadIntervalTimeout = 10;
	timeout.ReadTotalTimeoutMultiplier = 10;
	timeout.ReadTotalTimeoutConstant = 0;
	
	timeout.WriteTotalTimeoutMultiplier = 10;
	timeout.WriteTotalTimeoutConstant = 0;
	
	SetCommTimeouts(device,&timeout);
	
	return device;
}

size_t read_from_usb_device(HANDLE device, const char *buffer, size_t size)
{
	DWORD read_size = 0;
	if (!ReadFile(device, (char *)buffer, size, &read_size, 0)) {
		fprintf(stderr, "ReadFile fail: %d\n", GetLastError());
		return 0;
	}
	
	return read_size;
}

size_t write_to_usb_device(HANDLE device, char *buffer, size_t size)
{
	DWORD write_size = 0;
	if (!WriteFile(device, buffer, size, &write_size, 0)) {
		fprintf(stderr, "WriteFile fail: %d\n", GetLastError());
		return 0;
	}
	
	return write_size;
}

void close_usb_device(HANDLE device)
{
	CloseHandle(device);
}