/* 
 * File:   Client.cpp
 * Author: TheBeast
 * 
 * Created on 23. April 2016, 14:32
 */
#include <stdio.h>

#include "Client.hpp"

Client::Client(int port, char *host) {
	Client::portno = port;
	Client::sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0) Logger::log(DEBUG, "Socket not bound");
	server = gethostbyname(host);
	if(server == NULL) Logger::log(DEBUG, "Server not found");
	bzero((char *)&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portno);
	if(connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) Logger::log(DEBUG, "Socket not connected");
}

int Client::writeAngles(float *angles) {
	char buffer[256];
	snprintf(buffer, sizeof(buffer), "%f, %f, %f\n", angles[0], angles[1], angles[2]);
	int n = write(sockfd, buffer, strlen(buffer));
	if(n < 0){
		Logger::log(DEBUG, "Message not written\n");
		return -1;
	}
	return 0;
}

int Client::writeChar(char c) {
	
	if(write(sockfd, &c, strlen(&c)) < 0) Logger::log(DEBUG, "Char not written\n"); return -1;
	
	return 0;
}

int Client::writeValues(float angle, float rate, float dT) {
	
	char buffer[256];
	snprintf(buffer, sizeof(buffer), "Angle: %f, Rate: %f, dT: %f\n", angle, rate, dT);
	Logger::log(DEBUG, buffer);
	int n = write(sockfd, buffer, strlen(buffer));
	if(n < 0){
		Logger::log(DEBUG, "Message not written\n");
		return -1;
	}
	return 0;
	
}

int Client::writeRaw(float *acc, float* gyro) {
	char buffer[256];
	snprintf(buffer, sizeof(buffer), "%f, %f, %f, %f, %f, %f\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
	int n = write(sockfd, buffer, strlen(buffer));
	if(n < 0){
		Logger::log(DEBUG, "Message not written\n");
		return -1;
	}
	return 0;
}