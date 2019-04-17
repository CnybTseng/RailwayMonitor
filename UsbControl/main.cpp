#include <stdio.h>
#include <stdlib.h>
#include "usb_control.h"

int main(int argc, char *argv[])
{
	HANDLE device = open_usb_device(11, 9600, 8, 0, 1);
	if (device == INVALID_HANDLE_VALUE) return 0;
	
	// char check_status_cmd[] = {0x55, 0x00, 0x00, 0x00, 0x00};
	// size_t size = write_to_usb_device(device, check_status_cmd, sizeof(check_status_cmd));
	// if (!size) {
	// 	close_usb_device(device);
	// 	return 0;
	// }
	// 
	// char read_buffer[1024];
	// size = read_from_usb_device(device, read_buffer, sizeof(read_buffer));
	// if (!size) {
	// 	close_usb_device(device);
	// 	return 0;
	// }
	// 
	// printf("read back: ");
	// for (size_t i = 0; i < size; ++i) printf("%02x ", read_buffer[i]);
	// printf("\n");
	// 
	// if (read_buffer[5] != 0x02) {
		printf("the device is closed, open it now...\n");
		// char open_cmd[] = {0x55, 0x00, 0x00, 0x00, 0x02};
		char open_cmd[] = {0x55, 0x02, 0x00, 0x0A, 0x00};
		size_t size = write_to_usb_device(device, open_cmd, sizeof(open_cmd));
		if (!size) {
			close_usb_device(device);
			return 0;
		}
	// }
	
	// Sleep(3000);
	// 
	// char close_cmd[] = {0x55, 0x00, 0x00, 0x00, 0x01};
	// size = write_to_usb_device(device, close_cmd, sizeof(close_cmd));
	// if (!size) {
	// 	close_usb_device(device);
	// 	return 0;
	// }
	
	return 0;	
}