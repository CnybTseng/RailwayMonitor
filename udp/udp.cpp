#include "udp.h"

static void PrintSimpleLog(const char *msg);

// ----------------------------------------------------------------------------------
// Default constructor function of class udp_server.
// ----------------------------------------------------------------------------------
udp_server::udp_server()
{
	port = 32345;
}

// ----------------------------------------------------------------------------------
// Constructor function of class udp_server.
// ----------------------------------------------------------------------------------
udp_server::udp_server(
	unsigned short port_
)	{
	port = port_;
}

// ----------------------------------------------------------------------------------
// Copy constructor function of class udp_server.
// ----------------------------------------------------------------------------------
udp_server::udp_server(
		const udp_server &server
)	{
	port = server.port;
}

// ----------------------------------------------------------------------------------
// Assignment constructor function of class udp_server.
// ----------------------------------------------------------------------------------
udp_server &udp_server::operator=(
		const udp_server &other
)	{
	this->port = other.port;
	memmove(&this->sock, &other.sock, sizeof(SOCKET));
	memmove(&this->was_data, &other.was_data, sizeof(WSADATA));
	memmove(&this->local, &other.local, sizeof(struct sockaddr_in));
	memmove(&this->from, &other.from, sizeof(struct sockaddr_in));
	this->from_len = other.from_len;
	return *this;
}

// ----------------------------------------------------------------------------------
// Destructor function of class udp_server.
// ----------------------------------------------------------------------------------
udp_server::~udp_server()
{
	closesocket(sock);
	WSACleanup();
}

// ----------------------------------------------------------------------------------
// Open UDP communication.
// ----------------------------------------------------------------------------------
bool udp_server::open()
{
	/* Initiates use of the Winsock DLL by a process */
	if (WSAStartup(MAKEWORD(2, 2), &was_data)) { 
		printf("UDP socket init fail!\n");
		PrintSimpleLog("UDP socket init fail!\n");
		WSACleanup();
		return false;
	}

	/* Create a UDP socket */
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == INVALID_SOCKET) {
		printf("Create UDP socket fail.\n");
		PrintSimpleLog("Create UDP socket fail.\n");
		return false;
	}

	/* Set port reuse flag as 1 */
	int flag = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&flag, sizeof(flag)) == SOCKET_ERROR) {
		printf("Set socket option error: %d.\n", WSAGetLastError());
		char errmsg[1024];
		sprintf(errmsg, "Set socket option error: %d.\n", WSAGetLastError());
		PrintSimpleLog(errmsg);
	}

	/* Set broadcast flag as 1 */
	flag = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&flag, sizeof(flag)) == SOCKET_ERROR) {
		printf("Set socket option error: %d.\n", WSAGetLastError());
		char errmsg[1024];
		sprintf(errmsg, "Set socket option error: %d.\n", WSAGetLastError());
		PrintSimpleLog(errmsg);
	}

	/* Set non block flag as 1 */
	flag = 1;
	if (ioctlsocket(sock, FIONBIO, (unsigned long *)&flag) == SOCKET_ERROR) {
		printf("Set socket non block error: %d.\n", WSAGetLastError());
	}
	
	/* Bind UDP socket */
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.s_addr = INADDR_ANY;
	// local.sin_addr.s_addr = inet_addr("192.168.1.120"); 
	if (bind(sock, (struct sockaddr *)&local, sizeof(local)) == SOCKET_ERROR) {
		printf("Bind UDP socket error: %d.\n", WSAGetLastError());
		char errmsg[1024];
		sprintf(errmsg, "Bind UDP socket error: %d.\n", WSAGetLastError());
		PrintSimpleLog(errmsg);
		closesocket(sock);
		WSACleanup();
		return false;
	}

	return true;
}

// ----------------------------------------------------------------------------------
// Receive message from client.
// ----------------------------------------------------------------------------------
bool udp_server::receive(
		char *recv_buf,
		unsigned int recv_len
)	{
	/* Parameters check */
	assert(recv_buf);
	assert(recv_len > 0);
	from_len = sizeof(struct sockaddr);
	
	FD_ZERO(&rfd);
	FD_SET(sock, &rfd);
	timeout.tv_sec = 0;
	timeout.tv_usec = 10000;
	int select_ret = select(sock + 1, &rfd, 0, 0, &timeout);
	if (select_ret < 0) {
		char errmsg[1024];
		sprintf(errmsg, "listening error: %d.\n", WSAGetLastError());
		PrintSimpleLog(errmsg);
		return false;
	} else if (select_ret == 0) {
		char errmsg[1024];
		sprintf(errmsg, "time out: %d.\n", WSAGetLastError());
		PrintSimpleLog(errmsg);		
		return false;
	}
	
	/* Receive */
	int retval = recvfrom(sock, recv_buf, recv_len, 0, (struct sockaddr *)&from, &from_len);
	if (retval > 0) {
		// printf("Received %d bytes message from %s.\n%", retval, inet_ntoa(from.sin_addr));
		return true;
	} else if (retval == 0) {
		printf("The connection has been closed.\n");
		PrintSimpleLog("The connection has been closed.\n");
	} else {
		printf("UDP recvfrom error: %d.\n", WSAGetLastError());
		char errmsg[1024];
		sprintf(errmsg, "UDP recvfrom error: %d.\n", WSAGetLastError());
		PrintSimpleLog(errmsg);
		Sleep(50);
	}
	return false;
}

// ----------------------------------------------------------------------------------
// Receive message from and reply to client.
// ----------------------------------------------------------------------------------
bool udp_server::receive(
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
	/* Receive and reply */
	int retval = recvfrom(sock, recv_buf, recv_len, 0, (sockaddr *)&from, &from_len);
	if (retval > 0) {
		// printf("Received message from %s.\n", inet_ntoa(from.sin_addr));
		sendto(sock, reply_buf, reply_len, 0, (struct sockaddr*)&from, from_len);
		return true;
	} else if (retval == 0) {
		printf("The connection has been closed.\n");
	} else {
		printf("UDP recvfrom error: %d.\n", WSAGetLastError());
	}
	return false;
}

// ----------------------------------------------------------------------------------
// Close UDP communication.
// ----------------------------------------------------------------------------------
void udp_server::close()
{
	closesocket(sock);
	WSACleanup();
}

void PrintSimpleLog(const char *msg)
{
	FILE *fp = fopen("udp.txt", "a");
	fprintf(fp, msg);
	fclose(fp);
}