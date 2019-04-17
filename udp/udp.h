#ifndef _UDP_SERVER_H_
#define _UDP_SERVER_H_

#include <stdio.h>
#include <cassert>
#include <winsock.h>

class udp_server
{
public:
	/**
	 * Default constructor function of class udp_server.
	 */
	udp_server();
	/**
	 * Constructor function of class udp_server.
	 * @param[in] port_
	 */
	udp_server(
		unsigned short port_
	);
	/**
	 * Copy constructor function of class udp_server.
	 * @param[in] server Another UDP server.
	 */
	udp_server(
		const udp_server &server
	);
	/**
	 * Assignment constructor function of class udp_server.
	 * @param[in] server Another UDP server.
	 */
	udp_server &operator=(
		const udp_server &other
	);
	/**
	 * Destructor function of class udp_server.
	 */
	~udp_server();
	/**
	 * Open UDP communication.
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
	 * Close UDP communication.
	 */
	void close();
private:
	unsigned short port;
	SOCKET sock;
	WSADATA was_data;
	struct sockaddr_in local;
	struct sockaddr_in from;
	int from_len;
	fd_set rfd;
	struct timeval timeout;
};

#endif