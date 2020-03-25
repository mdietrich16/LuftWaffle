/* 
 * File:   Client.hpp
 * Author: TheBeast
 *
 * Created on 23. April 2016, 14:28
 */

#ifndef CLIENT_HPP
#define	CLIENT_HPP

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <cstdlib>
#include <sys/types.h>
#include <string.h>
#include <string>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include "Logger.hpp"

class Client {
	public:
		Client(int port, char *host);
		int writeAngles(float *angles);
		int writeValues(float angle, float rate, float dT);
		int writeRaw(float *acc, float* gyro);
		int writeChar(char c);
	private:
		int sockfd, portno, n;
		struct sockaddr_in serv_addr;
		struct hostent *server;
};

#endif	/* CLIENT_HPP */

