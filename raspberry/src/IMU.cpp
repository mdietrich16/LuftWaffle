/* 
 * File:   IMU.cpp
 * Author: TheBeast
 * 
 * Created on 12. März 2016, 14:28
 */

#include <cmath>

#include "IMU.hpp"
#include "Logger.hpp"

IMU::IMU() {
    offset[0] = 0.0;
    offset[1] = 0.0;
    offset[2] = 0.0;
    offset[3] = 0.0;
    offset[4] = 0.0;
    offset[5] = 0.0;
    offset[6] = 0.0;
    offset[7] = 0.0;
    offset[8] = 0.0;
}

IMU::~IMU() {
}

int IMU::init(accelRange rangeA, gyroRange rangeG, magRange rangeM, float alpha_, float beta_) {
    Logger::log(DEBUG, "IMU: Initializing IMU");

    rangea = rangeA;
    rangeg = rangeG;
    rangem = rangeM;

    alpha = alpha_;
    calpha = 1.0 - alpha;

    beta = beta_;

    i2c = I2C();

    Logger::log(DEBUG, "IMU: Enabling Accelerometer");
    i2c.writeByte(LSM303_ACCEL_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0b01100111);
    byte whoami;
    i2c.readByte(LSM303_ACCEL_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG1_A, &whoami);
    if (whoami != 0b01100111) {
	Logger::log(WARNING, "IMU: Unable to communicate with Accelerometer");
	return -1;
    }

    Logger::log(DEBUG, "IMU: Setting Accelerometer range to %iG", rangeA);
    switch (rangeA) {
	case RANGE_2G:
	    i2c.writeByte(LSM303_ACCEL_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0b00001000);
	    sensitivitya = SENSITIVITY_2G;
	    break;
	case RANGE_4G:
	    i2c.writeByte(LSM303_ACCEL_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0b00011000);
	    sensitivitya = SENSITIVITY_4G;
	    break;
	case RANGE_8G:
	    i2c.writeByte(LSM303_ACCEL_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0b00101000);
	    sensitivitya = SENSITIVITY_8G;
	    break;
	case RANGE_16G:
	    i2c.writeByte(LSM303_ACCEL_ADDRESS, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0b00111000);
	    sensitivitya = SENSITIVITY_16G;
	    break;
    }

    Logger::log(DEBUG, "IMU: Enabling Gyroscope");
    i2c.readByte(L3GD20H_GYRO_ADDRESS, GYRO_REGISTER_WHO_AM_I, &whoami);
    if (whoami != 0xD7) {
	Logger::log(WARNING, "IMU: Unable to communicate with Gyroscope");
	return -1;
    }
    i2c.writeByte(L3GD20H_GYRO_ADDRESS, GYRO_REGISTER_CTRL_REG1, 0b01011111);

    Logger::log(DEBUG, "IMU: Setting Gyroscope range to %iDPS", rangeG);
    switch (rangeG) {
	case RANGE_245DPS:
	    gRange = 245;
	    i2c.writeByte(L3GD20H_GYRO_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0b00000000);
	    sensitivityg = SENSITIVITY_254DPS;
	    break;
	case RANGE_500DPS:
	    gRange = 500;
	    i2c.writeByte(L3GD20H_GYRO_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0b00010000);
	    sensitivityg = SENSITIVITY_500DPS;
	    break;
	case RANGE_2000DPS:
	    gRange = 2000;
	    i2c.writeByte(L3GD20H_GYRO_ADDRESS, GYRO_REGISTER_CTRL_REG4, 0b00100000);
	    sensitivityg = SENSITIVITY_2000DPS;
	    break;
    }

    Logger::log(DEBUG, "IMU: Enabling Magnetometer");
    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRA_REG_M, 0b00011100);
    i2c.readByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRA_REG_M, &whoami);
    if (whoami != 0b00011100) {
	Logger::log(WARNING, "IMU: Unable to communicate with Magnetometer");
	return -1;
    }
    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_MR_REG_M, 0b00000000);

    Logger::log(DEBUG, "IMU: Setting Magnetometer range to %f gauss", ((double) rangeM) / 10);
    switch (rangeM) {
	case RANGE_1_3GAUSS:
	    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRB_REG_M, 0b00100000);
	    sensitivitym[0] = SENSITIVITY_1_3XY;
	    sensitivitym[1] = SENSITIVITY_1_3XY;
	    sensitivitym[2] = SENSITIVITY_1_3Z;
	    Logger::log(DEBUG, "IMU: Mag GAIN: %f, %f, %f", sensitivitym[0], sensitivitym[1], sensitivitym[2]);
	    break;
	case RANGE_1_9GAUSS:
	    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRB_REG_M, 0b01000000);
	    sensitivitym[0] = SENSITIVITY_1_9XY;
	    sensitivitym[1] = SENSITIVITY_1_9XY;
	    sensitivitym[2] = SENSITIVITY_1_9Z;
	    Logger::log(DEBUG, "IMU: Mag GAIN: %f, %f, %f", sensitivitym[0], sensitivitym[1], sensitivitym[2]);
	    break;
	case RANGE_2_5GAUSS:
	    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRB_REG_M, 0b01100000);
	    sensitivitym[0] = SENSITIVITY_2_5XY;
	    sensitivitym[1] = SENSITIVITY_2_5XY;
	    sensitivitym[2] = SENSITIVITY_2_5Z;
	    Logger::log(DEBUG, "IMU: Mag GAIN: %f, %f, %f", sensitivitym[0], sensitivitym[1], sensitivitym[2]);
	    break;
	case RANGE_4_0GAUSS:
	    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRB_REG_M, 0b10000000);
	    sensitivitym[0] = SENSITIVITY_4_0XY;
	    sensitivitym[1] = SENSITIVITY_4_0XY;
	    sensitivitym[2] = SENSITIVITY_4_0Z;
	    Logger::log(DEBUG, "IMU: Mag GAIN: %f, %f, %f", sensitivitym[0], sensitivitym[1], sensitivitym[2]);
	    break;
	case RANGE_4_7GAUSS:
	    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRB_REG_M, 0b10100000);
	    sensitivitym[0] = SENSITIVITY_4_7XY;
	    sensitivitym[1] = SENSITIVITY_4_7XY;
	    sensitivitym[2] = SENSITIVITY_4_7Z;
	    Logger::log(DEBUG, "IMU: Mag GAIN: %f, %f, %f", sensitivitym[0], sensitivitym[1], sensitivitym[2]);
	    break;
	case RANGE_5_6GAUSS:
	    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRB_REG_M, 0b11000000);
	    sensitivitym[0] = SENSITIVITY_5_6XY;
	    sensitivitym[1] = SENSITIVITY_5_6XY;
	    sensitivitym[2] = SENSITIVITY_5_6Z;
	    Logger::log(DEBUG, "IMU: Mag GAIN: %f, %f, %f", sensitivitym[0], sensitivitym[1], sensitivitym[2]);
	    break;
	case RANGE_8_1GAUSS:
	    i2c.writeByte(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_CRB_REG_M, 0b11100000);
	    sensitivitym[0] = SENSITIVITY_8_1XY;
	    sensitivitym[1] = SENSITIVITY_8_1XY;
	    sensitivitym[2] = SENSITIVITY_8_1Z;
	    Logger::log(DEBUG, "IMU: Mag GAIN: %f, %f, %f", sensitivitym[0], sensitivitym[1], sensitivitym[2]);
	    break;
    }

    //Logger::log(DEBUG, "IMU: Enabling barometric pressure sensor");

    return 0;
}

int IMU::read() {
    byte buff[6];
    i2c.readBlock(LSM303_ACCEL_ADDRESS, LSM303_REGISTER_ACCEL_OUT_X_L_A, buff, 6);


    accData.x = (((int16_t) (buff[0] | (buff[1] << 8)) >> 4) * sensitivitya);
    accData.y = -(((int16_t) (buff[2] | (buff[3] << 8)) >> 4) * sensitivitya);
    accData.z = -(((int16_t) (buff[4] | (buff[5] << 8)) >> 4) * sensitivitya);

    i2c.readBlock(L3GD20H_GYRO_ADDRESS, GYRO_REGISTER_OUT_X_L, buff, 6);

    gyroData.x += beta * ((((int16_t) (buff[0] | (buff[1] << 8))) * sensitivityg) - gyroData.x);
    gyroData.y += beta * ((-((int16_t) (buff[2] | (buff[3] << 8))) * sensitivityg) - gyroData.y);
    gyroData.z += beta * ((-((int16_t) (buff[4] | (buff[5] << 8))) * sensitivityg) - gyroData.z);

    i2c.readBlock(LSM303_MAG_ADDRESS, LSM303_REGISTER_MAG_OUT_X_H_M, buff, 6);

    magData.x = (((int16_t) (buff[1] | (buff[0] << 8))) / sensitivitym[0]) * 100; //MICROTESLA!!!!!!!!!!!!!!!
    magData.y = -(((int16_t) (buff[5] | (buff[4] << 8))) / sensitivitym[1]) * 100;
    magData.z = -(((int16_t) (buff[3] | (buff[2] << 8))) / sensitivitym[2]) * 100;
    return 0;
}

int IMU::calibrate() {
    float offsdummy[3] = {0};
    float beta_ = beta;
    beta = 1.0;
    
    Logger::log(DEBUG, "IMU: Test --- Test --- Test --- Test");
    
    gyroAngle.x = 0.0;
    gyroAngle.y = 0.0;
    gyroAngle.z = 0.0;

    gyroData.x = 0.0;
    gyroData.y = 0.0;
    gyroData.z = 0.0;
    
    int sample_num = 100;
    
    for (int i = 0; i < sample_num; i++) {
	read();
	offsdummy[0] += gyroData.x;
	offsdummy[1] += gyroData.y;
	offsdummy[2] += gyroData.z;
    }
    offset[0] = 0.012667; //-0.020332;
    offset[1] = 0.079961; //-0.010781;
    offset[2] = 0.048144; //-0.002778;
    offset[3] = offsdummy[0] / sample_num;
    offset[4] = offsdummy[1] / sample_num;
    offset[5] = offsdummy[2] / sample_num;
    offset[6] = -4.602825;
    offset[7] = -0.635921;
    offset[8] = -69.264458;
    beta = beta_;
    Logger::log(DEBUG, "Offsets: %f, %f, %f", offset[3], offset[4], offset[5]);
    return 0;
}

int IMU::initAngles() {

    accAngle.x = 0.0;
    accAngle.y = 0.0;
    accAngle.z = 0.0;

    gyroAngle.x = 0.0;
    gyroAngle.y = 0.0;
    gyroAngle.z = 0.0;

    gyroData.x = 0.0;
    gyroData.y = 0.0;
    gyroData.z = 0.0;

    accData.x = 0.0;
    accData.y = 0.0;
    accData.z = 0.0;

    magAngle = 0.0;

    read();
    processAcc();
    processMag();

    angle.x = accAngle.x;
    angle.y = accAngle.y;
    angle.z = magAngle;

    return 0;
}

int IMU::compensate() {

    //accData.x -= offset[0];
    //accData.y -= offset[1];
    //accData.z -= offset[2];

    //accData.x = accData.x * 1.046607 + accData.y * 0.023547 + accData.z * -0.014487;
    //accData.y = accData.x * 0.023547 + accData.y * 1.015572 + accData.z * -0.021206;
    //accData.z = accData.x * -0.014487 + accData.y * -0.021206 + accData.z * 0.944182;

    gyroData.x -= offset[3];
    gyroData.y -= offset[4];
    gyroData.z -= offset[5];

    magData.x -= offset[6];
    magData.y -= offset[7];
    magData.z -= offset[8];

    magData.x = magData.x * 1.040104 + magData.y * -0.008143 + magData.z * -0.039932;
    magData.y = magData.x * -0.008143 + magData.y * 0.988065 + magData.z * -0.036655;
    magData.z = magData.x * -0.039932 + magData.y * -0.036655 + magData.z * 0.895729;

    return 0;
}

int IMU::processAcc() {

    //angles[1] = -(atan2(accData.x, sqrt(pow(accData.y, 2) + pow(accData.z, 2)))) * (180 / M_PI);
    //angles[0] = (atan2(accData.y, sqrt(pow(accData.x, 2) + pow(accData.z, 2)))) * (180 / M_PI);

    accAngle.x = atan2(accData.y, sqrt(pow(accData.x, 2) + pow(accData.z, 2))) * (180.F / M_PI);
    accAngle.y = atan2(-accData.x, sqrt(pow(accData.z, 2) + pow(accData.y, 2))) * (180.F / M_PI);

    //float accelPitch = atanf((accelRealY)/sqrt(pow(accelRealX, 2) + pow(accelRealZ, 2)))*180.0F/PI;
    //float accelRoll = atan2f(-accelRealX, accelRealZ)*180.0F/PI;

    return 0;
}

int IMU::processMag() {

    magAngle = (atan2(magData.x, magData.y) * 180.0 / M_PI);
    //float numeratorYaw = magRealZ * cos(accelRoll * PI/180.0F) - magRealY * sin(accelRoll * PI/180.0F);
    //float denominatorYaw = magRealX * cos(accelPitch * PI/180.0F) + magRealY * sin(accelPitch * PI/180.0F)*sin(accelRoll * PI/180.0F) + magRealZ * sin(accelPitch * PI/180.0F)*cos(accelRoll * PI/180.0F);
    //float accelYaw = atan2f((numeratorYaw)/(denominatorYaw)) *180.0F/PI

    return 0;
}

int IMU::processGyro(float dT) {
    gyroAngle.x = angle.x + (gyroData.x * (dT / 1000));
    gyroAngle.y = angle.y + (gyroData.y * (dT / 1000));
    gyroAngle.z = gyroData.z * (dT / 1000);

    return 0;
}

IMU::vector<float> IMU::filter() {

    //			float a = 0.5 / (0.5 + (INTERVAL_MSEC / 1000.0));
    //0.980392
    angle.x = (alpha * gyroAngle.x) + (calpha * accAngle.x);
    angle.y = (alpha * gyroAngle.y) + (calpha * accAngle.y);
    angle.z = /*(0.5 * gyro[2]) + (0.5 * magAngle)*/magAngle;

    return angle;
}