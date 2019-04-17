#ifndef _USBCONTROL_H_
#define _USBCONTROL_H_

#include <windows.h>

HANDLE open_usb_device(int port, int baud_rate, int byte_size, int parity, int stop_bits);
size_t read_from_usb_device(HANDLE device, const char *buffer, size_t size);
size_t write_to_usb_device(HANDLE device, char *buffer, size_t size);
void close_usb_device(HANDLE device);

#endif