/* 
 * File:   Serial.hpp
 * Author: TheBeast
 *
 * Created on 14. Oktober 2016, 22:35
 */

#ifndef SERIAL_HPP
#define	SERIAL_HPP

#include <termios.h>
#include <fcntl.h>
#include "Logger.hpp"
#include <unistd.h>

class Serial {
public:
	Serial();
	virtual ~Serial();
	int init(char* port, speed_t baud);
	int print(char* msg);
	int print(char* msg, int len);
	int print(float msg);
	int write(char msg);
	int read(char* buff, int length);
	int available();
	int close();
private:
	int fd;
	
};

#endif	/* SERIAL_HPP */

