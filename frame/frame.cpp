#include <cassert>
#include <cmath>
#include <algorithm>
#include <cinttypes> 
#include "frame.h"
#include "codec.h"
#include "crc.h"

#define LZO_COMPRESS									(1)

static void PrintSimpleLog(const char *msg);

// ----------------------------------------------------------------------------------
// Default constructor function of class frame_receiver.
// ----------------------------------------------------------------------------------
frame_receiver::frame_receiver()
{
	
}

// ----------------------------------------------------------------------------------
// Constructor function of class frame_receiver.
// ----------------------------------------------------------------------------------
frame_receiver::frame_receiver(
	int mode_,
	unsigned short port
)	{
	mode = mode_;
	if (mode == UDP_SERVER_USE) {
		user = udp_server(port);
	} else if (mode == TCP_SERVER_USE) {
		tser = tcp_server(port);
	}
	recv_buf = new char[MAX_RECEIVE_BUFFER_SIZE];
	assert(recv_buf);
	recombine_data = new char[MAX_COMPRESS_DATA_SIZE];
	assert(recombine_data);
	InitializeCriticalSection(&frame_header_cs);
	InitializeCriticalSection(&mix_frame_data_cs);
	InitializeCriticalSection(&frame_data_cs);
	InitializeCriticalSection(&image_queue_cs);
	stop_thread_flag = false;
	counter = 0;
	discard = 0;
}

// ----------------------------------------------------------------------------------
// Destructor function of class frame_receiver.
// ----------------------------------------------------------------------------------
frame_receiver::~frame_receiver()
{
	if (recv_buf) {
		delete [] recv_buf;
		recv_buf = NULL;
	}
	
	if (recombine_data) {
		delete [] recombine_data;
		recombine_data = NULL;
	}
	
	for (int i = 0; i < MAX_CAMERA_NUMBER; i++) {
		/* Free data memory of frame_header_vector */
		if (frame_header_vector[i].capacity() > 0) {
			// printf("Free frame_header_vector.\n");
			std::vector<frame_header_t>().swap(frame_header_vector[i]);
		}
		/* Free data memory of mix_frame_data_vector */
		if (mix_frame_data_vector[i].capacity() > 0) {
			std::vector<frame_data_t>::iterator mix_data_it;
			for (mix_data_it = mix_frame_data_vector[i].begin(); mix_data_it != mix_frame_data_vector[i].end(); mix_data_it++) {
				if (mix_data_it->data != NULL) {
					delete [] mix_data_it->data;
					mix_data_it->data = NULL;
				}
			}
			// printf("Free mix_frame_data_vector.\n");
			std::vector<frame_data_t>().swap(mix_frame_data_vector[i]);
		}
		/* Free data memory of frame_data_vector */
		if (frame_data_vector[i].capacity() > 0) {
			std::vector<std::vector<frame_data_t>>::iterator data_it;
			for (data_it = frame_data_vector[i].begin(); data_it != frame_data_vector[i].end(); data_it++) {
				if ((*data_it).capacity() > 0) {
					std::vector<frame_data_t>::iterator frame_slice_it;
					for (frame_slice_it = (*data_it).begin(); frame_slice_it != (*data_it).end(); frame_slice_it++) {
						if (frame_slice_it->data != NULL) {
							delete [] frame_slice_it->data;
							frame_slice_it->data = NULL;
						}
					}
					// printf("Free frame_slice_vector.\n");
					std::vector<frame_data_t>().swap(*data_it);
				}
			}
			// printf("Free frame_data_vector.\n");
			std::vector<std::vector<frame_data_t>>().swap(frame_data_vector[i]);
		}
		/* Free data memory of image_queue */
		if (image_queue[i].size() > 0) {
			while (image_queue[i].size() > 0) {
				char *pure_image_data = image_queue[i].front();
				if (pure_image_data) {
					delete [] pure_image_data;
					pure_image_data = NULL;
				}
				image_queue[i].pop();
			}
		}
	}
	
	DeleteCriticalSection(&frame_header_cs);
	DeleteCriticalSection(&mix_frame_data_cs);
	DeleteCriticalSection(&frame_data_cs);
	DeleteCriticalSection(&image_queue_cs);
	
	if (mode == UDP_SERVER_USE) {
		user.close();
	} else if (mode == TCP_SERVER_USE) {
		tser.close();
	}
}

// ----------------------------------------------------------------------------------
// Init frame receiver.
// ----------------------------------------------------------------------------------
void frame_receiver::init(
	int mode_,
	unsigned short port
)	{
	mode = mode_;
	if (mode == UDP_SERVER_USE) {
		user = udp_server(port);
	} else if (mode == TCP_SERVER_USE) {
		tser = tcp_server(port);
	}
	recv_buf = new char[MAX_RECEIVE_BUFFER_SIZE];
	assert(recv_buf);
	recombine_data = new char[MAX_COMPRESS_DATA_SIZE];
	assert(recombine_data);
	InitializeCriticalSection(&frame_header_cs);
	InitializeCriticalSection(&mix_frame_data_cs);
	InitializeCriticalSection(&frame_data_cs);
	InitializeCriticalSection(&image_queue_cs);
	stop_thread_flag = false;
	counter = 0;
	discard = 0;
}

// ----------------------------------------------------------------------------------
// Open frame receiver.
// ----------------------------------------------------------------------------------
bool frame_receiver::open()
{
	if (mode == UDP_SERVER_USE) {
		return user.open();
	} else if (mode == TCP_SERVER_USE) {
		return tser.open();
	}
	return false;
}

/**
 * Print hexadecimal numbers
 * @param[in] buf Hexadecimal number buffer.
 * @param[in] len Length of hexadecimal number buffer.
 * @return void.
 */
static void print_hex(
	unsigned char *buf,
	unsigned int len
)	{
	for (unsigned int i = 0; i < len; i++)
		printf("%02X ", buf[i]);
	printf("\n");
}

/**
 * Print frame header.
 * @param[in] header Frame header.
 * @return void.
 */
static void print_header(
	frame_header_t header
)	{
	printf("Camera: %u\n", header.camera_index);
	printf("Number of data frames: %u\n", header.ndata_frames);
	printf("Width: %u\n", header.width);
	printf("Height: %u\n", header.height);
	printf("Standard: %u\n", header.standard);
	printf("Timestamp: %u\n", header.timestamp);
}

// ----------------------------------------------------------------------------------
// Put raw mix frame in vector.
// ----------------------------------------------------------------------------------
void frame_receiver::put_frame_in_vector()
{
	char frame_type;
	unsigned int crc;
	memmove(&frame_type, recv_buf, 1);
	/* Simply put the frame in vector */
	if (frame_type == FRAME_TYPE_HEADER) {
		frame_header_t frame_header;
		memcpy(&frame_header, (unsigned char *)recv_buf, sizeof(frame_header_t));		
		/* Network word order to host word order */
		frame_header.ndata_frames = ntohs(frame_header.ndata_frames);
		frame_header.width = ntohs(frame_header.width);
		frame_header.height = ntohs(frame_header.height);
		frame_header.timestamp = ntohs(frame_header.timestamp);	
		frame_header.crc = ntohl(frame_header.crc);
		/* CRC check */
		crc = crc32buf(recv_buf, sizeof(frame_header_t) - sizeof(unsigned int));		
		if (frame_header.crc != crc) {
			PrintSimpleLog("frame header crc error\n");
			return;
		}		
		std::vector<frame_header_t>::iterator header_it;
		EnterCriticalSection(&frame_header_cs);
		header_it = frame_header_vector[frame_header.camera_index].begin();
		frame_header_vector[frame_header.camera_index].insert(header_it, frame_header);
		LeaveCriticalSection(&frame_header_cs);
	} else if (frame_type == FRAME_TYPE_DATA) {
		frame_data_t frame_data;
		memcpy(&frame_data, recv_buf, DATA_BIT_BIAS);
		/* Network word order to host word order */
		frame_data.package_index = ntohs(frame_data.package_index);
		frame_data.image_data_size = ntohl(frame_data.image_data_size);
		frame_data.timestamp = ntohs(frame_data.timestamp);

		if (frame_header_vector[frame_data.camera_index].size() > 0) {
			int find_match_head = 0;
			std::vector<frame_header_t>::iterator header_it;
			EnterCriticalSection(&frame_header_cs);
			for (header_it = frame_header_vector[frame_data.camera_index].end() - 1;
				header_it >= frame_header_vector[frame_data.camera_index].begin(); header_it--) {
				if (frame_data.timestamp == header_it->timestamp) {
					find_match_head = 1;
					break;
				}
			}	
			LeaveCriticalSection(&frame_header_cs);
			if (find_match_head == 0) return;
		}
		
		memcpy(&frame_data.crc, recv_buf + DATA_BIT_BIAS + DATA_PACKAGE_SIZE, sizeof(unsigned int));
		frame_data.crc = ntohl(frame_data.crc);
		/* CRC check */
		crc = crc32buf(recv_buf, DATA_BIT_BIAS + DATA_PACKAGE_SIZE); 
		if (frame_data.crc != crc) {
			PrintSimpleLog("frame data crc error\n");
			return;
		}
		
		/* Copy image slice data */
		frame_data.data = new char[frame_data.image_data_size];	
		memcpy(frame_data.data, recv_buf + DATA_BIT_BIAS, frame_data.image_data_size);
		std::vector<frame_data_t>::iterator data_it;
		EnterCriticalSection(&mix_frame_data_cs);
		data_it = mix_frame_data_vector[frame_data.camera_index].begin();
		mix_frame_data_vector[frame_data.camera_index].insert(data_it, frame_data);
		LeaveCriticalSection(&mix_frame_data_cs);
	} else {
		printf("Frame type unknown!\n");
		PrintSimpleLog("Frame type unknown!\n");
	}
}

// ----------------------------------------------------------------------------------
// UDP or TCP server.
// ----------------------------------------------------------------------------------
DWORD WINAPI udp_tcp_server(
	PVOID args
)	{
	frame_receiver *receiver = (frame_receiver *)args;
	/* UDP server */
	if (receiver->mode == UDP_SERVER_USE) {		
		while (receiver->stop_thread_flag == false) {			
			memset(receiver->recv_buf, 0, MAX_RECEIVE_BUFFER_SIZE);
			receiver->recv_buf[MAX_RECEIVE_BUFFER_SIZE - 1] = '\0';				
			if (receiver->user.receive(receiver->recv_buf, MAX_RECEIVE_BUFFER_SIZE)) {
				receiver->put_frame_in_vector();
			}
		}
	} else if (receiver->mode == TCP_SERVER_USE) {		/* TCP server */
		while (receiver->stop_thread_flag == false) {
			memset(receiver->recv_buf, 0, MAX_RECEIVE_BUFFER_SIZE);
			receiver->recv_buf[MAX_RECEIVE_BUFFER_SIZE - 1] = '\0';
			if (receiver->tser.receive(receiver->recv_buf, MAX_RECEIVE_BUFFER_SIZE)) {
				receiver->put_frame_in_vector();
			}
		}
	}
	return 0;
}

// ----------------------------------------------------------------------------------
// UDP or TCP server thread.
// ----------------------------------------------------------------------------------
bool frame_receiver::udp_tcp_server_thread()
{
	HANDLE parser_handle = CreateThread(NULL, 0, LPTHREAD_START_ROUTINE(udp_tcp_server), this, 0, 0);
	if (!parser_handle) {
		printf("Create thread fail!\n");
		return false;
	}
	return true;
}

// ----------------------------------------------------------------------------------
// Maintain frame header vector size.
// ----------------------------------------------------------------------------------
void frame_receiver::maintain_frame_header_vector_status(
	int camera_index
)	{
	EnterCriticalSection(&frame_header_cs);
	while (frame_header_vector[camera_index].size() > MAX_CACHE_FRAME_NUMBER) {	
		printf("Remove the oldest frame header.\n");
		frame_header_vector[camera_index].pop_back();
	}
	LeaveCriticalSection(&frame_header_cs);
}

// ----------------------------------------------------------------------------------
// Maintain mix frame data vector size.
// ----------------------------------------------------------------------------------
void frame_receiver::maintain_mix_frame_data_vector_status(
	int camera_index
)	{
	EnterCriticalSection(&mix_frame_data_cs);
	while (mix_frame_data_vector[camera_index].size() > MAX_PACKAGE_NUMBER_PER_FRAME * MAX_CACHE_FRAME_NUMBER) {	
		printf("Remove the oldest frame data.\n");
		std::vector<frame_data_t>::iterator it = mix_frame_data_vector[camera_index].end() - 1;
		if (it->data != NULL) {
			delete [] it->data;
			it->data = NULL;
		}
		mix_frame_data_vector[camera_index].pop_back();
	}
	LeaveCriticalSection(&mix_frame_data_cs);
}

// ----------------------------------------------------------------------------------
// Maintain image data queue size.
// ----------------------------------------------------------------------------------
void frame_receiver::maintain_image_data_queue_status(
	int camera_index
)	{
	EnterCriticalSection(&image_queue_cs);
	while (image_queue[camera_index].size() > MAX_CACHE_FRAME_NUMBER) {
		if (0 == discard % 100) {
			printf("Remove the oldest pure image data[%"PRId64":%"PRId64"].\n", discard, counter);
		}
		char *pure_image_data = image_queue[camera_index].front();
		if (pure_image_data) {
			delete [] pure_image_data;
			pure_image_data = NULL;
		}
		image_queue[camera_index].pop();
		discard++;
	}
	LeaveCriticalSection(&image_queue_cs);
}

// ----------------------------------------------------------------------------------
// Recombine frame slice.
// ----------------------------------------------------------------------------------
void frame_receiver::recombine_frame_data(
	int camera_index
)	{
	std::vector<frame_data_t>::iterator mix_data_it;
	EnterCriticalSection(&mix_frame_data_cs);
	/* For every frame slice */
	for (mix_data_it = mix_frame_data_vector[camera_index].begin();
		mix_data_it != mix_frame_data_vector[camera_index].end();) {
		bool find_matching_frame = false;
		std::vector<std::vector<frame_data_t>>::iterator data_it;
		EnterCriticalSection(&frame_data_cs);
		for (data_it = frame_data_vector[camera_index].begin();
			data_it != frame_data_vector[camera_index].end(); data_it++) {
			/* If this frame slice belong to some existing frame */
			if ((*data_it)[0].timestamp == mix_data_it->timestamp) {
				(*data_it).push_back(*mix_data_it);
				find_matching_frame = true;				
				break;
			}
		}
		/* Not found matching frame */
		if (!find_matching_frame) {
			/* If the frame slice vector is full */
			if (frame_data_vector[camera_index].size() > MAX_CACHE_FRAME_NUMBER) {
				data_it = frame_data_vector[camera_index].end() - 1;
				std::vector<frame_data_t>::iterator frame_slice_it;
				for (frame_slice_it = (*data_it).begin(); frame_slice_it != (*data_it).end(); frame_slice_it++) {
					if (frame_slice_it->data != NULL) {
						delete [] frame_slice_it->data;
						frame_slice_it->data = NULL;
					}
				}
				frame_data_vector[camera_index].pop_back();
			}
			/* New a special frame slice vector */
			std::vector<frame_data_t> new_frame_data;
			new_frame_data.push_back(*mix_data_it);
			data_it = frame_data_vector[camera_index].begin();
			frame_data_vector[camera_index].insert(data_it, new_frame_data);
		}
		LeaveCriticalSection(&frame_data_cs);
		mix_data_it = mix_frame_data_vector[camera_index].erase(mix_data_it);
	}
	LeaveCriticalSection(&mix_frame_data_cs);
}

// ----------------------------------------------------------------------------------
// Comparison function of frame_data_t data.
// ----------------------------------------------------------------------------------
static bool compare(
	const frame_data_t &a,
	const frame_data_t &b
)	{
	return a.package_index < b.package_index;
}

// ----------------------------------------------------------------------------------
// Raw data codec.
// ----------------------------------------------------------------------------------
DWORD WINAPI codec(
	PVOID args
)	{
	frame_receiver *receiver = (frame_receiver *)args;
	while (receiver->stop_thread_flag == false) {
		/* For every camera */
		for (int i = 0; i < MAX_CAMERA_NUMBER; i++) {
			receiver->maintain_frame_header_vector_status(i);
			receiver->maintain_mix_frame_data_vector_status(i);
			receiver->maintain_image_data_queue_status(i);
			
			EnterCriticalSection(&receiver->frame_header_cs);
			EnterCriticalSection(&receiver->mix_frame_data_cs);
			/* Not found any frame from camera i */
			if (receiver->frame_header_vector[i].size() <= 0 ||
				receiver->mix_frame_data_vector[i].size() <= 0) {
				LeaveCriticalSection(&receiver->mix_frame_data_cs);
				LeaveCriticalSection(&receiver->frame_header_cs);
				continue;
			}
			LeaveCriticalSection(&receiver->mix_frame_data_cs);
			LeaveCriticalSection(&receiver->frame_header_cs);
			
			receiver->recombine_frame_data(i);

			std::vector<frame_header_t>::iterator header_it;
			std::vector<std::vector<frame_data_t>>::iterator it;
			EnterCriticalSection(&receiver->frame_header_cs);
			/* For every frame header of camera i */
			for (header_it = receiver->frame_header_vector[i].end() - 1;
				header_it >= receiver->frame_header_vector[i].begin(); header_it--) {
				std::vector<std::vector<frame_data_t>>::iterator data_it;
				EnterCriticalSection(&receiver->frame_data_cs);

				/* For every frame slice vector */
				for (data_it = receiver->frame_data_vector[i].begin();
					data_it != receiver->frame_data_vector[i].end(); data_it++) {
					/* Timestamp isn't match */
					if ((*data_it)[0].timestamp != header_it->timestamp) {
						continue;
					}
					/* Frame slice vector isn't complete */
					if ((*data_it).size() != header_it->ndata_frames) {
						continue;
					}
					/* Record resolution */
					receiver->widths[i] = header_it->width;
					receiver->heights[i] = header_it->height;
					std::vector<frame_data_t>::iterator frame_slice_it;
					/* Data size */
					unsigned int data_size = 0;
					for (frame_slice_it = (*data_it).begin(); frame_slice_it != (*data_it).end(); frame_slice_it++) {
						data_size += frame_slice_it->image_data_size;
					}

					unsigned int write_length = 0;
					/* Frame slice sort */
					std::sort((*data_it).begin(), (*data_it).end(), compare);								
					/* For every frame slice */
					for (frame_slice_it = (*data_it).begin(); frame_slice_it != (*data_it).end(); frame_slice_it++) {	
						memmove(receiver->recombine_data + write_length, frame_slice_it->data, frame_slice_it->image_data_size);
						write_length += frame_slice_it->image_data_size;
					}
					
					if (LZO_COMPRESS == header_it->null) {
						/* Decompress frame data with LZO */
						lzo_uint out_len = header_it->width * header_it->height * 3;
						lzo_uint new_len = out_len;
						char *decompress_data = new char[out_len];
						decoder((lzo_bytep)receiver->recombine_data, data_size, (lzo_bytep)decompress_data, new_len);	
						
						/* Push pure image data in queue */
						EnterCriticalSection(&receiver->image_queue_cs);
						receiver->image_queue[i].push(decompress_data);
						LeaveCriticalSection(&receiver->image_queue_cs);
					} else {
						char *uncompress_data = new char[write_length];
						memmove(uncompress_data, receiver->recombine_data, write_length);
						EnterCriticalSection(&receiver->image_queue_cs);
						receiver->image_queue[i].push(uncompress_data);
						LeaveCriticalSection(&receiver->image_queue_cs);
					}
					
					/* Remove frame data */			
					for (frame_slice_it = (*data_it).begin(); frame_slice_it != (*data_it).end(); frame_slice_it++) {
						if (frame_slice_it->data != NULL) {
							delete [] frame_slice_it->data;
							frame_slice_it->data = NULL;
						}
					}
					receiver->frame_data_vector[i].erase(data_it);
					/* Remove frame header */
					header_it = receiver->frame_header_vector[i].erase(header_it);
					receiver->counter++;
					break;
				}

				LeaveCriticalSection(&receiver->frame_data_cs);
				if (header_it == receiver->frame_header_vector[i].begin()) {
					break;
				}
			}
			LeaveCriticalSection(&receiver->frame_header_cs);
		}
	}

	return 0;
}

// ----------------------------------------------------------------------------------
// Raw data codec thread.
// ----------------------------------------------------------------------------------
bool frame_receiver::codec_thread()
{
	HANDLE codec_handle = CreateThread(NULL, 0, LPTHREAD_START_ROUTINE(codec), this, 0, 0);
	if (!codec_handle) {
		printf("Create thread fail!\n");
		return false;
	}
	return true;
}

// ----------------------------------------------------------------------------------
// Run frame receiver.
// ----------------------------------------------------------------------------------
bool frame_receiver::run()
{
	if (!udp_tcp_server_thread()) {
		return false;
	}
	if (!codec_thread()) {
		return false;
	}
	return true;
}

// ----------------------------------------------------------------------------------
// Get pure image data.
// ----------------------------------------------------------------------------------
bool frame_receiver::get(
	unsigned short camera_index,
	char *data,
	unsigned short &width,
	unsigned short &height
)	{
	/* Parameters check */
	assert(camera_index >= 0 && camera_index < MAX_CAMERA_NUMBER);
	assert(data);

	EnterCriticalSection(&image_queue_cs);
	if (image_queue[camera_index].size() > 0) {
		char *pure_image_data = image_queue[camera_index].front();
		memmove(data, pure_image_data, widths[camera_index] * heights[camera_index] * 2);
		width = widths[camera_index];
		height = heights[camera_index];
		/* Remove pure image data */
		if (pure_image_data) {
			delete [] pure_image_data;
			pure_image_data = NULL;
		}
		image_queue[camera_index].pop();
		LeaveCriticalSection(&image_queue_cs);
		return true;
	}
	LeaveCriticalSection(&image_queue_cs);
	return false;
}

// ----------------------------------------------------------------------------------
// Convert semi YUV420 to uint16_t.
// ----------------------------------------------------------------------------------
void frame_receiver::semi_yuv422_touint16(
	char *raw_data,
	uint16_t *u16_data,
	int32_t length
)	{
	assert(raw_data);
	assert(u16_data);
	assert(length > 0);
	int32_t u_shift = length / 2;
	int32_t v_shift = u_shift * 3 / 2;
	int32_t uv_count = 0;
	// Data arrangement: UY VY UY VY ...
	for (int32_t i = 0; i < u_shift; i++) {
		uint16_t low_8bits = 0x00ff & raw_data[i];	// Y component.
		uint16_t hig_8bits;
		if (i % 2 == 0) {							// U component.
			hig_8bits = raw_data[u_shift + uv_count]  & 0x7f;
		} else {									// V component.
			hig_8bits = raw_data[v_shift + uv_count]  & 0x7f;
			uv_count++;
		}
		u16_data[i] = (hig_8bits << 7) + low_8bits;
	}
}

// ----------------------------------------------------------------------------------
// Convert 640x480 raw data format to uint16_t.
// ----------------------------------------------------------------------------------
#if 0
void frame_receiver::raw640_touint16(
	char *raw_data,
	uint16_t *u16_data,
	int32_t length
)	{
	assert(raw_data);
	assert(u16_data);
	assert(length > 0);
	int32_t npixels = length / 2;
	for (int32_t i = 0; i < npixels; i++) {
		uint16_t low_8bits = 0x00ff & raw_data[i * 2];
		uint16_t hig_8bits = raw_data[i * 2 + 1]  & 0x7f;
		u16_data[i] = (hig_8bits << 7) + low_8bits;
	}
}
#else
void frame_receiver::raw640_touint16(
	char *raw_data,
	uint16_t *u16_data,
	int32_t length
	)	{
	assert(raw_data);
	assert(u16_data);
	assert(length > 0);
	int32_t npixels = length / 2;
	for (int32_t i = 0; i < npixels; i++) {
		uint16_t low_8bits = 0x00ff & raw_data[i * 2];
		uint16_t hig_8bits = raw_data[i * 2 + 1] & 0x7f;
		u16_data[i] = (hig_8bits << 8) + low_8bits;
	}
}
#endif

// ----------------------------------------------------------------------------------
// Recombine raw data.
// ----------------------------------------------------------------------------------
void frame_receiver::recombine_raw_data(
	char *raw_data,
	uint16_t *u16_data,
	int32_t length
)	{
#ifdef IPC384_USE
	semi_yuv422_touint16(raw_data, u16_data, length);	
#else
	raw640_touint16(raw_data, u16_data, length);
#endif
}

// ----------------------------------------------------------------------------------
// Replace zero lines with neighborhood lines.
// ----------------------------------------------------------------------------------
void frame_receiver::fill_zero_lines(
	uint16_t *data,
	int32_t width,
	int32_t height
)	{
	assert(data);
	assert(width > 0);
	assert(height > 0);
	// First and second line of first frame.
	memmove(data, data + 2 * width, 2 * width * sizeof(uint16_t));
}

// ----------------------------------------------------------------------------------
// Parse board temperature.
// ----------------------------------------------------------------------------------
double frame_receiver::parse_board_temperature(
	int32_t reading_voltage
)	{
	const int32_t voltage = 1992;
	const int32_t reference_voltage = 1800;
	const int32_t temperature_base = 33;
	const int32_t voltage_base = 1850;
	const double voltage_change_rate = 14.8;
	double tmp = reference_voltage + (double)voltage * reading_voltage / pow(2.0, 14) - voltage / 2.0;
	double board_temperature = temperature_base - (voltage_base - tmp) / voltage_change_rate;
	return board_temperature;
}

// ----------------------------------------------------------------------------------
// Parse environment temperature.
// ----------------------------------------------------------------------------------
float frame_receiver::parser_envir_temper(unsigned short sensor_reading)
{
	int sign;
	const float TEMP_PER_COUNT = 0.0625f;
	enum {POSITIVE = 0};
	float env_temper;
	
	sign = (sensor_reading & 0x800) >> 11;
	
	if (POSITIVE == sign) {
		env_temper = sensor_reading * TEMP_PER_COUNT;
	} else {
		env_temper = -((~(sensor_reading - 0x0001)) & 0x0FFF) * TEMP_PER_COUNT;
	}
	
	return env_temper;
}

// ----------------------------------------------------------------------------------
// Stop frame receiver.
// ----------------------------------------------------------------------------------
void frame_receiver::stop()
{
	stop_thread_flag = true;
}

void PrintSimpleLog(const char *msg)
{
	FILE *fp = fopen("frame.txt", "a");
	fprintf(fp, msg);
	fclose(fp);
}