#include "tcp.h"

// ----------------------------------------------------------------------------------
// Default constructor function of class tcp_server.
// ----------------------------------------------------------------------------------
tcp_server::tcp_server()
{
	port  = 42345;
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.S_un.S_addr = INADDR_ANY;
	from_len = sizeof(from);
}

// ----------------------------------------------------------------------------------
// Constructor function of class tcp_server.
// ----------------------------------------------------------------------------------
tcp_server::tcp_server(
	unsigned short port_
)	{
	port  = port_;
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.S_un.S_addr = INADDR_ANY;
	from_len = sizeof(from);
}

// ----------------------------------------------------------------------------------
// Copy constructor function of class tcp_server.
// ----------------------------------------------------------------------------------
tcp_server::tcp_server(
	const tcp_server &server
)	{
	port  = server.port;
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.S_un.S_addr = INADDR_ANY;
	from_len = sizeof(from);
}

// ----------------------------------------------------------------------------------
// Destructor function of class tcp_server.
// ----------------------------------------------------------------------------------
tcp_server::~tcp_server()
{
	closesocket(sock);
	WSACleanup();
}

// ----------------------------------------------------------------------------------
// Open TCP communication.
// ----------------------------------------------------------------------------------
bool tcp_server::open()
{
	/* Initiates use of the Winsock DLL by a process */
	if (WSAStartup(MAKEWORD(2, 1), &was_data)) { 
		printf("TCP socket init fail!\n");
		WSACleanup();
		return false;
	}
	/* Create a UDP socket */
	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock == INVALID_SOCKET) {
		printf("Create UDP socket fail.\n");
		return false;
	}
	/* Bind UDP socket */
	if (bind(sock, (struct sockaddr*)&local, sizeof(local)) == SOCKET_ERROR) {
		printf("Bind UDP socket error!\n");
		closesocket(sock);
		return false;
	}
	/* Begin listening */
	if (listen(sock, 5) == SOCKET_ERROR) {
		printf("TCP listening error");
		closesocket(sock);
		return false;
	}
	return true;
}

// ----------------------------------------------------------------------------------
// Receive message from client.
// ----------------------------------------------------------------------------------
bool tcp_server::receive(
	char *recv_buf,
	unsigned int recv_len
)	{
	/* Parameters check */
	assert(recv_buf);
	assert(recv_len > 0);
	/* Accept a connection in a socket */
	client_sock = accept(sock, (struct sockaddr*)&from, &from_len);
	if (client_sock == INVALID_SOCKET) {
		printf("TCP accept error!\n");
		return false;
	}
	/* Receive */
	if (recv(client_sock, recv_buf, recv_len, 0)) {
		// printf("Received message from %s.\n%", inet_ntoa(from.sin_addr));
		closesocket(client_sock);
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------------
// Receive message from and reply to client.
// ----------------------------------------------------------------------------------
bool tcp_server::receive(
	char *recv_buf,
	unsigned int recv_len,
	char *reply_buf,
	unsigned int reply_len
)	{
	/* Parameters check */
	assert(recv_buf);
	assert(recv_len > 0);
	assert(reply_buf);
	assert(reply_len > 0);
	/* Accept a connection in a socket */
	client_sock = accept(sock, (struct sockaddr*)&from, &from_len);
	if (client_sock == INVALID_SOCKET) {
		printf("TCP accept error!\n");
		return false;
	}
	/* Receive and reply */
	if (recv(client_sock, recv_buf, recv_len, 0)) {
		// printf("Received message from %s.\n%", inet_ntoa(from.sin_addr));
		send(client_sock, reply_buf, reply_len, 0);
		closesocket(client_sock);
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------------
// Close TCP communication.
// ----------------------------------------------------------------------------------
void tcp_server::close()
{
	closesocket(sock);
	WSACleanup();
}