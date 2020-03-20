/* 
 * File: IMU.hpp
 * Author: TheBeast
 *
 * Created on 12. März 2016, 14:28
 */

#ifndef IMU_HPP
#define	IMU_HPP

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "I2C.hpp"

typedef enum {
	LSM303_ACCEL_ADDRESS = 0b00011001,
	LSM303_MAG_ADDRESS = 0b00011110,
	L3GD20H_GYRO_ADDRESS = 0b01101011,
	BMP180_P_ADDRESS = 0b01110111
} sensorAdresses_t;

typedef enum
{                                                     // DEFAULT    TYPE
  LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
  LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
  LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
  LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
  LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
  LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
  LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
  LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
  LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
  LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
  LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
  LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
  LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
  LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
  LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
  LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
  LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
  LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
  LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
  LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
  LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
  LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
  LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
  LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
  LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
  LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
  LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
  LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
  LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
  LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
} lsm303AccelRegisters_t;

typedef enum
{
  LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
  LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
  LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
  LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
  LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
  LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
  LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
  LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
  LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
  LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
  LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
  LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
  LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
  LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
  LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
} lsm303MagRegisters_t;

typedef enum
{                                             // DEFAULT    TYPE
  GYRO_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
  GYRO_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
  GYRO_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
  GYRO_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
  GYRO_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
  GYRO_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
  GYRO_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
  GYRO_REGISTER_OUT_TEMP            = 0x26,   //            r
  GYRO_REGISTER_STATUS_REG          = 0x27,   //            r
  GYRO_REGISTER_OUT_X_L             = 0x28,   //            r
  GYRO_REGISTER_OUT_X_H             = 0x29,   //            r
  GYRO_REGISTER_OUT_Y_L             = 0x2A,   //            r
  GYRO_REGISTER_OUT_Y_H             = 0x2B,   //            r
  GYRO_REGISTER_OUT_Z_L             = 0x2C,   //            r
  GYRO_REGISTER_OUT_Z_H             = 0x2D,   //            r
  GYRO_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
  GYRO_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
  GYRO_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
  GYRO_REGISTER_INT1_SRC            = 0x31,   //            r
  GYRO_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
  GYRO_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
  GYRO_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
  GYRO_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
  GYRO_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
  GYRO_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
  GYRO_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
} gyroRegisters_t; 
	
typedef enum {
	RANGE_2G = 2,
	RANGE_4G = 4,
	RANGE_8G = 8,
	RANGE_16G = 16
} accelRange;
#define SENSITIVITY_2G						0.001F
#define SENSITIVITY_4G						0.002F
#define SENSITIVITY_8G						0.004F
#define SENSITIVITY_16G						0.012F

typedef enum {
	RANGE_245DPS = 254,
	RANGE_500DPS = 500,
	RANGE_2000DPS = 2000
} gyroRange;

#define SENSITIVITY_254DPS					0.00875F
#define SENSITIVITY_500DPS					0.0175F
#define SENSITIVITY_2000DPS					0.07F

typedef enum {
	RANGE_1_3GAUSS = 13,
	RANGE_1_9GAUSS = 19,
	RANGE_2_5GAUSS = 25,
	RANGE_4_0GAUSS = 40,
	RANGE_4_7GAUSS = 47,
	RANGE_5_6GAUSS = 56,
	RANGE_8_1GAUSS = 81
} magRange;

#define SENSITIVITY_1_3XY						1100.0F
#define SENSITIVITY_1_3Z						980.0F
#define SENSITIVITY_1_9XY						855.0F
#define SENSITIVITY_1_9Z						760.0F
#define SENSITIVITY_2_5XY						670.0F
#define SENSITIVITY_2_5Z						600.0F
#define SENSITIVITY_4_0XY						450.0F
#define SENSITIVITY_4_0Z						400.0F
#define SENSITIVITY_4_7XY						400.0F
#define SENSITIVITY_4_7Z						350.0F
#define SENSITIVITY_5_6XY						330.0F
#define SENSITIVITY_5_6Z						295.0F
#define SENSITIVITY_8_1XY						230.0F
#define SENSITIVITY_8_1Z						205.0F

class IMU {
public:
	template <typename T> struct vector
    {
      T x, y, z;
    };
	IMU();
	~IMU();
	int init(accelRange rangeA, gyroRange rangeG, magRange rangeM, float alpha_ ,float beta_);
	int calibrate();
	int initAngles();
	int compensate();
	//int readFIFOA();
	//int readFIFOG();
	//int readFIFOM();
	int read();
	int processAcc();
	int processMag();
	int processGyro(float dT);
	int altitudeBMP();
	vector<float> filter();
	vector<float> accData;
	vector<float> accAngle;
	vector<float> magData;
	float magAngle;
	vector<float> gyroData;
	vector<float> gyroAngle;
	vector<float> angle;
	//vector<float> altData;
	
	int gRange;
private:
	I2C i2c;
	float offset[9];
	float sensitivitya;
	float sensitivityg;
	float sensitivitym[3];
	float alpha;
	float calpha;		    //complement of alpha = 1-alpha
	float beta;
	accelRange rangea;
	gyroRange rangeg;
	magRange rangem;
};

#endif	/* IMU_HPP */