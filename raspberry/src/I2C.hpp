/* 
 * File:   I2C.hpp
 * Author: TheBeast
 *
 * Created on 13. April 2016, 15:17
 */

#ifndef I2C_HPP
#define	I2C_HPP

typedef unsigned char byte;
#define I2C_SLAVE			0x0703

//#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "Logger.hpp"
#include <unistd.h>

class I2C {
public:
	I2C();
	int writeByte(int addr, byte reg, byte data);
	int readByte(int addr, byte reg, byte *data);
	int writeBlock(int addr, byte startReg, byte *data, int bytes);
	int readBlock(int addr, byte startReg, byte *data, int bytes);
private:
	int fd;
};

#endif	/* I2C_HPP */

