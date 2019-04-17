#ifndef _TCP_SERVER_H_
#define _TCP_SERVER_H_

#include <stdio.h>
#include <cassert>
#include <winsock.h>

class tcp_server
{
public:
	/**
	 * Default constructor function of class tcp_server.
	 */
	tcp_server();
	/**
	 * Constructor function of class tcp_server.
	 * @param[in] port_
	 */
	tcp_server(
		unsigned short port_
	);
	/**
	 * Copy constructor function of class tcp_server.
	 * @param[in] server Another UDP server.
	 */
	tcp_server(
		const tcp_server &server
	);
	/**
	 * Destructor function of class tcp_server.
	 */
	~tcp_server();
	/**
	 * Open TCP communication.
	 */
	bool open();
	/**
	 * Receive message from client.
	 * @param[in] recv_buf Receive buffer.
	 * @param[in] recv_len Length of receive buffer.
	 * @return true if success, otherwise false.
	 */
	bool receive(
		char *recv_buf,
		unsigned int recv_len
	);
	/**
	 * Receive message from and reply to client.
	 * @param[in] recv_buf Receive buffer.
	 * @param[in] recv_len Length of receive buffer.
	 * @param[in] reply_buf Reply buffer.
	 * @param[in] reply_len Length of reply buffer.
	 * @return true if success, otherwise false.
	 */
	bool receive(
		char *recv_buf,
		unsigned int recv_len,
		char *reply_buf,
		unsigned int reply_len
	);
	/**
	 * Close TCP communication.
	 */
	void close();
private:
	unsigned short port;
	SOCKET sock;
	SOCKET client_sock;
	WSADATA was_data;
	struct sockaddr_in local;
	struct sockaddr_in from;
	int from_len;
};

#endif