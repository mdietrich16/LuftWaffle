/* 
 * File:   Serial.cpp
 * Author: TheBeast
 * 
 * Created on 14. Oktober 2016, 22:35
 */

#include <sys/ioctl.h>

#include "Serial.hpp"

using namespace std;

Serial::Serial() {
}

Serial::~Serial() {
}

int Serial::init(char* port, speed_t baud) {
	
	struct termios toptions;
	memset(&toptions, 0, sizeof(toptions));
	Logger::log(DEBUG, "SERIAL: Opening device file, getting and setting attributes.");
	fd = open(port, O_RDWR | O_NOATIME);
	
	if(fd == -1) {
		Logger::log(WARNING, "SERIAL: Error opening device file.");
		return -1;
	}
	
	if(tcgetattr(fd, &toptions) < 0) {
		Logger::log(WARNING, "SERIAL: Error getting device attributes.");
		return -1;
	}
	
	cfsetospeed(&toptions, baud);
	cfsetispeed(&toptions, baud);
	
	toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;
	
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    toptions.c_oflag &= ~OPOST;
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
	
	tcflush(fd, TCIFLUSH);
	if(tcsetattr(fd, TCSANOW, &toptions) != 0) {
		Logger::log(WARNING, "SERIAL: Error setting device attributes.");
		return -1;
	}
	return 0;
}

int Serial::close() {
	Logger::log(DEBUG, "SERIAL: Closing file descriptor.");
	::close(fd);
	return 0;
}

int Serial::print(char *msg) {
	
	int n = ::write(fd, msg, strlen(msg));
//	printf("%s, %d\n", msg, n);
	if(n <= 0) {
		Logger::log(WARNING, "SERIAL: Error writing message. Error #: %d", n);
		return -1;
	}
	return n;
}

int Serial::print(char *msg, int len) {
	
	int n = ::write(fd, msg, len);
//	printf("%s, %d\n", msg, n);
	if(n <= 0) {
		Logger::log(WARNING, "SERIAL: Error writing message. Error #: %d", n);
		return -1;
	}
	return n;
}

int Serial::print(float f) {
    
    char msg[4];
    memcpy(msg, &f, 4);
    return print(msg, 4);
}

int Serial::write(char msg) {
	
	int n = ::write(fd, &msg, 1);
//	printf("%s, %d\n", msg, n);
	if(n <= 0) {
		Logger::log(WARNING, "SERIAL: Error writing message. Error #: %d", n);
		return -1;
	}
	return n;
}

int Serial::read(char* buffer, int len) {
//	fcntl(fd, F_SETFL, FNDELAY);
	int n = 1;
	n = ::read(fd, buffer, len);		//ACHTUNG!!! array pointer übergeben verliert die Länge!
	if(n <= 0) {
		Logger::log(WARNING, "SERIAL: Nothing read.");
	}
	return n;
}

int Serial::available() {
    int bytes_available = 0;
    ioctl(fd, FIONREAD, &bytes_available);
    return bytes_available;
}