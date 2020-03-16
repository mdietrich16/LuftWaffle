/*
 * File:   main.cpp
 * Author: TheBeast
 *
 * Created on 12. März 2016, 13:38
 */

#include <cstdlib>
#include <csignal>
#include <cstdio>
#include <stdint.h>
#include <string>
#include <iostream>
#include <cmath>
#include <sys/time.h>
#include <sys/socket.h>
#include "IMU.hpp"
#include "Logger.hpp"
#include "Client.hpp"
#include "Serial.hpp"

#define INTERVAL_MSEC			2
#define USE_PROCESSING			false
#define RUNTIME_SEC			60
#define RUNTIME				RUNTIME_SEC * 1000 / INTERVAL_MSEC
#define END_CONDITION			true			//true = run for RUNTIME/1000 seconds, false = run indefinitely

#define COMM_OK					(char)0b10000000			//to signal OK
#define COMM_FAIL				(char)0b11111111			//something went wrong
#define COMM_START				(char)0b11000000			//start communication
#define COMM_END				(char)0b10100000			//end communication
#define COMM_DATA				(char)0b10000011			//indicates new data
#define COMM_DATA_2				(char)0b11111100			//initiates 2nd part of data
#define COMM_INFO				(char)0b10000100			//indicates status msg following

using namespace std;

Serial serial;
sig_atomic_t inted = 0;

    
double millis() {
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    return (double) (time.tv_sec * 1000.0) + ((double) time.tv_nsec / 1000000.0);
}

void end() {
    serial.write(COMM_INFO);
    serial.write(COMM_END);
    usleep(10000);
    serial.close();
    Logger::log(INFO, "Closing program");
    Logger::log(INFO, "####################################");
    Logger::close();
    printf("Exiting...");
}

void sig(int args) {
    inted = 1;
}

int main(int argc, char** argv) {

    Logger::init(DEBUG);
    Logger::log(INFO, "####################################");
    Logger::log(INFO, "Starting program");
    double dt;			    //Delta time = Zeit zwischen zwei loops
    int count = 0;		    //Wie viele Iterationen
    double time = 0;		    //Aktuelle Zeit
    double t = 0;		    //Temporäre Buffervariable für dei Zeit um zwei millis()-Aufrufe zu verhindern
    bool loop = true;		    //Na was wohl
    char port[20] = "/dev/ttyACM0"; //Port vom Arduino --- Hängt ab von "native"(ttyACM0) oder "programming"(ttyAMA0)
    char msg = 0;		    //Buffer
    char message[32];
    int checksum = 0;
    
    atexit(end);								//Finish-up wenn das Programm endet
    signal(SIGINT, sig);							//Jeder Interrupt wird abgefangen
    signal(SIGABRT, sig);
    signal(SIGTERM, sig);
    signal(SIGQUIT, sig);
    
    serial = Serial();
    serial.init(port, B2000000);

    serial.write(COMM_INFO);
    serial.write(COMM_END);

    IMU imu = IMU();
    imu.init(RANGE_2G, RANGE_245DPS, RANGE_1_3GAUSS, 0.9, 0.2);
    imu.calibrate();
    
    Logger::flush();
    
    IMU::vector<float> angle;
    uint16_t frac_angle;
    
    serial.write(COMM_START);
    while (msg != COMM_START && (inted == 0)) {
	printf("%i : %i : %c\n", serial.read(&msg, 1), msg, msg);
	usleep(50000);
    }
    
    usleep(50000);
    serial.write(COMM_INFO);		//Write PID coefficients
    serial.write(COMM_DATA_2);
    serial.print(1.f);
    serial.print(0.0000f);
    serial.print(0.4f);
    
    imu.initAngles();
    time = millis();
#if USE_PROCESSING
    char hostname[20] = "192.168.178.21";
    Client client = Client(50099, hostname);
#endif

    while (loop && (inted == 0)) {
	t = millis()
	dt = (t - time);

	if (dt >= INTERVAL_MSEC) {
	    count++;

	    time = t;

	    imu.read();
	    //imu.compensate();
	    imu.processAcc();
	    imu.processMag();
	    imu.processGyro(dt);
	    angle = imu.filter();

	    Logger::log(DEBUG, "ACC: X: %f, Y: %f, Z: %f", imu.accAngle.x, imu.accAngle.y, imu.accAngle.z);
	    Logger::log(DEBUG, "GYRO: X: %f, Y: %f, Z: %f", imu.gyroData.x, imu.gyroData.y, imu.gyroData.z);
	    Logger::log(INFO, "ANGLES: X: %f, Y: %f, Z: %f, Delta time: %f", angle.x, angle.y, angle.z, dt);
	    
	    if ((count % 20) == 1) {
		Logger::flush();
	    }
	    
	    
	    checksum = 0;
	    serial.write(COMM_DATA);
	    frac_angle = (uint16_t) (int) round((angle.x + 180.) * 182.044444444444444444);
	    checksum = (checksum + frac_angle) % 255;
	    serial.write(frac_angle >> 8);
	    serial.write(frac_angle & 0b0000000011111111);
	    frac_angle = (uint16_t) (int) round((angle.y + 180.) * 182.044444444444444444);
	    checksum = (checksum + frac_angle) % 255;
	    serial.write(frac_angle >> 8);
	    serial.write(frac_angle & 0b0000000011111111);
	    frac_angle = (uint16_t) (int) round((angle.z + 180.) * 182.044444444444444444);
	    checksum = (checksum + frac_angle) % 255;
	    serial.write(frac_angle >> 8);
	    serial.write(frac_angle & 0b0000000011111111);
	    serial.write(checksum);
	    
	    
	    serial.write(COMM_DATA_2);
	    serial.print(imu.gyroData.x);
	    serial.print(imu.gyroData.y);
	    serial.print(imu.gyroData.z);
	    serial.write((abs((int)imu.gyroData.x) + abs((int)imu.gyroData.y) + abs((int)imu.gyroData.z)) % 255);
	    
	    
	    //printf("Acc: %f, %f\n", imu.accAngle.x, imu.accAngle.y);
	    //printf("Gyro: %f, %f, %f\n", imu.gyroData.x, imu.gyroData.y, imu.gyroData.z);
//	    printf("Ang: %f, %f\n", angle.x, angle.y);
	    memset(message, 0, 32);
	    int available = serial.available();
	    if(available > 0 && serial.read(message, min(available, 30)) > 0) {
		printf("%s", message);
//		printf("Ang: %f, %f\n", angle.x, angle.y);
	    }
#if USE_PROCESSING
	    //if(client.writeValues(aangle[0], data[3], dt/1000) != 0) {    ONLY KALMANCOMPLEMENTARY
//		if(client.writeRaw(acc, gyro)) {
	    if((count % 40) == 0) {
		if (client.writeAngles(angle)) {
		    loop = false;
		}
	    }
#else
	    if((count >= (RUNTIME)) && END_CONDITION){
		loop = false;
	    }
#endif
	}
    }
#if USE_PROCESSING
    client.writeChar('!');
#endif
}