#ifndef _FRAME_RECEIVER_H_
#define _FRAME_RECEIVER_H_

#include <windows.h>
#include <queue>
#include <vector>
#include "udp.h"
#include "tcp.h"

#ifdef _MSC_VER
#if _MSC_VER < 1600
typedef char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
#else
#include <cstdint>
#endif
#else
#include <cstdint>
#endif

#define MAX_COMPRESS_DATA_SIZE			(1024 * 1024 * 2)
#define MAX_CAMERA_NUMBER				(8)
#define MAX_CACHE_FRAME_NUMBER			(4)
#define MAX_PACKAGE_NUMBER_PER_FRAME	(19)
#define MAX_RECEIVE_BUFFER_SIZE			(1024 * 1024 * 2)
#define FRAME_TYPE_HEADER				(0x5A)
#define FRAME_TYPE_DATA					(0x6A)
#define DATA_PACKAGE_SIZE				(0x00008000U)
#define DATA_BIT_BIAS					(0x0000000CU)

/**
 * \enum frame receiver work mode.
 */
enum {
	UDP_SERVER_USE = 0,	// TCP mode.
	TCP_SERVER_USE = 1	// UDP mode.
};

/**
 * \typedef struct frame_header_t
 * \brief Data structure for the frame header.
 */
typedef struct tag_frame_header {
	unsigned char frame_type;
	unsigned char camera_index;
	unsigned short ndata_frames;
	unsigned short width;
	unsigned short height;
	unsigned char standard;	// PAL or NTSC
	unsigned char null;
	unsigned short timestamp;
	unsigned int crc;
	/* Assignment operator overload function */
	tag_frame_header &operator=(const tag_frame_header &other) {
		this->frame_type = other.frame_type;
		this->camera_index = other.camera_index;
		this->ndata_frames = other.ndata_frames;
		this->width = other.width;
		this->height = other.height;
		this->standard = other.standard;
		this->timestamp = other.timestamp;
		return *this;
	}
}frame_header_t;

/**
 * \typedef struct frame_data_t
 * \brief Data structure for the frame data.
 */
typedef struct tag_frame_data{
	unsigned char frame_type;
	unsigned char camera_index;
	unsigned short package_index;
	unsigned int image_data_size;
	unsigned short timestamp;
	unsigned char null[2];
	char *data;		// 4 bytes, zero padding
	unsigned int crc;
	/* Constructor function */
	tag_frame_data() {
		memset(this, 0, sizeof(tag_frame_data));
	}
	/* Assignment operator overload function */
	tag_frame_data &operator=(const tag_frame_data &other) {
		this->frame_type = other.frame_type;
		this->camera_index = other.camera_index;
		this->package_index = other.package_index;
		this->image_data_size = other.image_data_size;
		this->timestamp = other.timestamp;
		this->data = other.data;	/* Copy address */
		return *this;
	}
}frame_data_t;

/**
 * \typedef class frame_receiver
 * \brief Structure for the frame_receiver.
 */
class frame_receiver
{
public:
	/**
	 * Default constructor function of class frame_receiver.
	 */
	frame_receiver();
	/**
	 * Constructor function of class frame_receiver.
	 * @param[in] mode_ Communication mode.
	 * @param[in] port Communication port.
	 */
	frame_receiver(
		int mode_,
		unsigned short port
	);
	/**
	 * Destructor function of class frame_receiver.
	 */
	~frame_receiver();
	/**
	 * Init frame receiver.
	 * @param[in] mode_ Communication mode.
	 * @param[in] port Communication port.
	 * @return void.
	 */
	void init(
		int mode_,
		unsigned short port
	);
	/**
	 * Open frame receiver.
	 * @return True if success, otherwise false.
	 */
	bool open();
	/**
	 * Run frame receiver.
	 * @return True if success, otherwise false.
	 */
	bool run();
	/**
	 * Get pure image data.
	 * @param[in] camera_index Thermal camera index.
	 * @param[out] data Image data buffer.
	 * @param[out] width Image width.
	 * @param[out] height Image height.
	 * @return True if success, otherwise false.
	 */
	bool get(
		unsigned short camera_index,
		char *data,
		unsigned short &width,
		unsigned short &height
	);
		/**
	 * Convert semi YUV420 to uint16_t.
	 * @param[in] raw_data YUV420 data.
	 * @param[out] u16_data uint16_t data.
	 * @param[in] length Data byte length.
	 * @return void.
	 */
	void semi_yuv422_touint16(
		char *raw_data,
		uint16_t *u16_data,
		int32_t length
	);
	/**
	 * Convert 640x480 raw data format to uint16_t.
	 * @param[in] raw_data YUV420 data.
	 * @param[out] u16_data uint16_t data.
	 * @param[in] length Data byte length.
	 * @return void.
	 */
	void raw640_touint16(
		char *raw_data,
		uint16_t *u16_data,
		int32_t length
	);
	/**
	 * Recombine raw data.
	 * @param[in] raw_data YUV420 data.
	 * @param[out] u16_data uint16_t data.
	 * @param[in] length Data byte length.
	 * @return void.
	 */
	void recombine_raw_data(
		char *raw_data,
		uint16_t *u16_data,
		int32_t length
	);
	/**
	 * Replace zero lines with neighborhood lines.
	 * @param[in,out] data Thermal image data.
	 * @param[in] width Image width.
	 * @param[in] height Image height.
	 * @return void.
	 */
	void fill_zero_lines(
		uint16_t *data,
		int32_t width,
		int32_t height
	);
	/**
	 * Parse board temperature.
	 * @param[in] reading_voltage Output of temperature sensor.
	 * @return board temperature.
	 */
	double parse_board_temperature(
		int32_t reading_voltage
	);
	/**
	 * Parse environment temperature.
	 * @param sensor_reading.
	 * @return environment temperature.
	 */
	float parser_envir_temper(unsigned short sensor_reading);
	/**
	 * Stop frame receiver.
	 */
	void stop();
private:
	/**
	 * Put raw mix frame in vector.
	 * @return void.
	 */
	void put_frame_in_vector();
	/**
	 * UDP or TCP server.
	 * @param[in] args.
	 * @return void *.
	 */
	friend DWORD WINAPI udp_tcp_server(
		PVOID args
	);
	/**
	 * UDP or TCP server thread.
	 * @return True if success, otherwise false.
	 */
	bool udp_tcp_server_thread();
	/**
	 * Maintain frame header vector size.
	 * @return void.
	 */
	void maintain_frame_header_vector_status(
		int camera_index
	);
	/**
	 * Maintain mix frame data vector size.
	 * @return void.
	 */
	void maintain_mix_frame_data_vector_status(
		int camera_index
	);
	/**
	 * Maintain image data queue size.
	 * @return void.
	 */
	void maintain_image_data_queue_status(
		int camera_index
	);
	/**
	 * Recombine frame slice.
	 * @return void.
	 */
	void recombine_frame_data(
		int camera_index
	);
	/**
	 * Raw data codec.
	 * @param[in] args.
	 * @return void *.
	 */
	friend DWORD WINAPI codec(
		PVOID args
	);
	/**
	 * Raw data codec thread.
	 * @return True if success, otherwise false.
	 */
	bool codec_thread();
private:
	char *recv_buf;		/* If recv_buf is behind tser, run error will happen */
	char *recombine_data;	/* Recombine compressed data */
	std::vector<frame_header_t> frame_header_vector[MAX_CAMERA_NUMBER];	/* Raw frame header */
	std::vector<frame_data_t> mix_frame_data_vector[MAX_CAMERA_NUMBER];	/* Mix raw frame data */
	std::vector<std::vector<frame_data_t>> frame_data_vector[MAX_CAMERA_NUMBER];	/* Sorted raw frame data */
	std::queue<char *> image_queue[MAX_CAMERA_NUMBER];	/* Raw images */
	unsigned short widths[MAX_CAMERA_NUMBER];
	unsigned short heights[MAX_CAMERA_NUMBER];
	CRITICAL_SECTION frame_header_cs;
	CRITICAL_SECTION mix_frame_data_cs;
	CRITICAL_SECTION frame_data_cs;
	CRITICAL_SECTION image_queue_cs;
	int mode;			/* Communication mode */
	udp_server user;	/* UDP server */
	tcp_server tser;	/* TCP server */
	bool stop_thread_flag;
	unsigned long long counter;
	unsigned long long discard;
};

#endif