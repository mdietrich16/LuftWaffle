/* 
 * File:   I2C.cpp
 * Author: TheBeast
 * 
 * Created on 13. April 2016, 15:17
 */

#include "I2C.hpp"

I2C::I2C() {
	char file[15];
	snprintf(file, 14, "/dev/i2c-1");
	fd = open(file, O_RDWR);
	if(fd < 0) {
		Logger::log(CRITICAL, "I2C: Cant open I2C device file");
	}
}

int I2C::writeByte(int addr, byte reg, byte data) {
	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
		Logger::log(ERROR, "I2C - writeByte: Setting address failed");
	}
	byte buf[2] = {reg, data};
	if(write(fd, buf, 2) < 2) {
		Logger::log(ERROR, "I2C - writeByte: Writing failed");
	}
	return 0;
}

int I2C::readByte(int addr, byte reg, byte *data) {
	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
		Logger::log(ERROR, "I2C - readByte: Setting address failed");
	}
	if(write(fd, &reg, 1) < 1) {
		Logger::log(ERROR, "I2C - readByte: Writing failed");
	}
	if(read(fd, data, 1) < 1) {
		Logger::log(ERROR, "I2C - readByte: Reading failed");
	}
	return 0;
}

int I2C::readBlock(int addr, byte startReg, byte *data, int bytes) {
	startReg = startReg | 0b10000000;
	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
		Logger::log(ERROR, "I2C - readBlock: Setting address failed");
	}
	if(write(fd, &startReg, 1) < 1) {
		Logger::log(ERROR, "I2C - readBlock: Writing failed");
	}
	if(read(fd, data, bytes) < bytes) {
		Logger::log(ERROR, "I2C - readBlock: Reading failed");
	}
	return 0;
}

int I2C::writeBlock(int addr, byte startReg, byte* data, int bytes) {
	startReg = startReg | 0b10000000;
	if(ioctl(fd, I2C_SLAVE, addr) < 0) {
		Logger::log(ERROR, "I2C - writeByte: Setting address failed");
	}
	if(write(fd, &startReg, 1) < 1) {
		Logger::log(ERROR, "I2C - writeByte: Writing register failed");
	}
	if(write(fd, data, bytes) < bytes) {
		Logger::log(ERROR, "I2C - writeByte: Writing failed");
	}
	return 0;
}
