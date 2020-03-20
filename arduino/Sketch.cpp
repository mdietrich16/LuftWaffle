/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

//#include "PID_v1.h"
#include <PID.h>
#include <Servo.h>
//Beginning of Auto generated function prototypes by Atmel Studio
void setupInterrupts();
bool setupComm();
void setupESCs();
void setupPID();
void serialInitialize();
void processRCData();
void processPilot();
void parseSerial();
void updateESCs();
void writeESCs();
void throttleHandler();
void rollHandler();
void pitchHandler();
void yawHandler();
void functionHandler();
//End of Auto generated function prototypes by Atmel Studio



Servo ESCs[4];
PID qc_p_a;
//PID qc_p_r;
//PID qc_r_c;
//PID qc_y_c;

const bool debug = true;
#define M_E             2.718281828459045235360287471352    //Euler's  number e
#define SQRT_2          1.4142135623730950488016887242097   //sqrt(2)
#define INT_TO_DEG		0.0054931640625                    //180 / 65536
#define DEG_TO_RAD      0.017453292519943295769236908       //M_PI / 180
#define RAD_TO_DEG      57.29577951308232087679815481       //180 / M_PI

//MSB == 1 shows coniguration message

#define COMM_OK         0b10000000      //to signal OK							:128
#define COMM_FAIL       0b11111111      //something went wrong					:255
#define COMM_START      0b11000000      //start communication					:192
#define COMM_END        0b10100000      //end communication						:160
#define COMM_DATA       0b10000011      //indicates new data following		:131
#define COMM_DATA_2     0b11111100      //indicates new data transmitted	:252
#define COMM_INFO       0b10000100      //indicates status msg following	:132

int qc_throttle_target = 1000;
float qc_target_a[3] = {0, 0, 0};         //pitch, roll, yaw
float qc_actual_a[3];                     //pitch, roll, yaw
float qc_error_a[3];                      //pitch, roll, yaw
float qc_target_r[3] = {0, 0, 0};         //pitch, roll, yaw
float qc_actual_r[3];                     //pitch, roll, yaw
float qc_error_r[3];                      //pitch, roll, yaw
int qc_esc_us[4];                         //ESCs are in order FR, RR, RL, FL on pins 3, 4, 5, 6

long qc_time;
long qc_last_time;
bool qc_enabled = false;

float qc_p_a_k[3];

unsigned long rc_start_time[5] = {0L, 0L, 0L, 0L, 0L};
volatile unsigned short rc_val[5] = {1000, 1500, 1500, 1500};
volatile unsigned short rc_val_last[4];
int rc_input[4];
int rc_output[4];
unsigned long rc_now = 0L;
byte rc_switch_state[2];
int rc_index;
volatile short rc_switch_val[8] = {0};
const float rc_smooth = 0.5;

const int qc_esc_pin1 = 3;                //ESCs are in order FR, RR, RL, FL on pins 3, 4, 5, 6
const int rc_throttlePin = 10;
const int rc_rollPin = 8;
const int rc_pitchPin = 9;
const int rc_yawPin = 11;
const int rc_functionPin = 12;

byte comm_response;
byte comm_message;
int comm_checksum;
long comm_last_time;
uint16_t comm_angles[3];

float sin_p;
float cos_p;
float sin_r;
float cos_r;
float numerator;
float denominator;

//############################################################################

void setup() {
	Serial.begin(2000000);
	while (!Serial){delay(10);}
	setupInterrupts();
	
	qc_time = micros();
	while(micros() - qc_time < 20000);

	while(!(rc_switch_val[0] > 1650));
	qc_enabled = true;
	
	while(!setupComm()) {		                //Blocks until OK comes in
		Serial.print(rc_switch_val[0]);
	}
	
	serialInitialize();
	setupPID();
	setupESCs();

	qc_time = micros();
	while(micros() - qc_time < 5000);
	updateESCs();
	writeESCs();
	
	parseSerial();
	processRCData();
	
	qc_time = micros();
	qc_p_a.update(qc_time);
	
	processPilot();
}

//############################################################################

void loop() {
	qc_time = micros();
	parseSerial();
	processRCData();
	
	qc_p_a.update(qc_time);
	
	processPilot();
	
	if ((qc_time - qc_last_time > 2000)) {
		qc_last_time = qc_time;
		updateESCs();
		writeESCs();
	}
	
	if (debug && (qc_time - comm_last_time > 20000)) {
		comm_last_time = qc_time;
		Serial.print(qc_actual_a[1]);
		Serial.print('\t');
		Serial.print(qc_actual_r[1]);
		Serial.print('\t');
		Serial.print(qc_target_a[1]);
		Serial.print('\t');
		Serial.print(qc_error_a[1]);
		Serial.print('\t');
		Serial.print(qc_esc_us[0]);
		Serial.print('\t');
		Serial.print(qc_esc_us[2]);
		Serial.print('\n');
	}
}

//############################################################################

void setupInterrupts() {
	pinMode(rc_throttlePin, INPUT_PULLUP);
	attachInterrupt(rc_throttlePin, throttleHandler, CHANGE);
	pinMode(rc_yawPin, INPUT_PULLUP);
	attachInterrupt(rc_yawPin, yawHandler, CHANGE);
	pinMode(rc_rollPin, INPUT_PULLUP);
	attachInterrupt(rc_rollPin, rollHandler, CHANGE);
	pinMode(rc_pitchPin, INPUT_PULLUP);
	attachInterrupt(rc_pitchPin, pitchHandler, CHANGE);
	pinMode(rc_functionPin, INPUT);
	attachInterrupt(rc_functionPin, functionHandler, CHANGE);
	rc_input[0] = 1000;
	rc_input[1] = 1500;
	rc_input[2] = 1500;
	rc_input[3] = 1500;
}

bool setupComm() {
	Serial.write(COMM_START);
	while (Serial.available() < 1){delay(10);}
	comm_response = Serial.read();
	while(comm_response == COMM_INFO || comm_response == COMM_END) {
		comm_response = Serial.read();
	}
	return comm_response == COMM_START;
}

void setupESCs() {
	for (int i = 0; i < 4; i++) {
		//pinMode(qc_esc_pin1 + i, OUTPUT);
		ESCs[i].attach(qc_esc_pin1 + i, 1000, 2000);
	}
	
	delay(1000);
	
	for (int i = 0; i < 4; i++) {
		ESCs[i].writeMicroseconds(0);
	}
	
	delay(500);
	
	for (int i = 0; i < 4; i++) {
		ESCs[i].writeMicroseconds(1100);
	}
}

void setupPID() {
	qc_p_a = PID(&(qc_actual_a[1]), &(qc_target_a[1]), &(qc_error_a[1]), qc_p_a_k[0], qc_p_a_k[1], qc_p_a_k[2], -400.F, 400.F, 2000);
}

void serialInitialize() {
	while (Serial.available() < 2){delay(10);}
	if (Serial.read() == COMM_INFO && Serial.read() == COMM_DATA_2) {
		for(int i = 0; i < 3; i++) {
			while (Serial.available() < 4){delay(10);}
			long buff = 0;
			for (int c = 0; c < 4; c++) {
				buff += (Serial.read() << c*8);
			}
			memcpy(&qc_p_a_k[i], &buff, 4);
		}
		Serial.print(qc_p_a_k[0]);
		Serial.print('\t');
		Serial.print(qc_p_a_k[1], 5);
		Serial.print('\t');
		Serial.print(qc_p_a_k[2], 3);
		Serial.print('\n');
	} else {
		Serial.write(COMM_FAIL);
		Serial.write(COMM_END);
		Serial.print("Parsing error\n");
	}
}

void processRCData() {
	for (int i = 0; i < 4; i++) {
		rc_input[i] = ((abs(rc_val[i] - rc_input[i]) > 550) ? rc_input[i] : rc_val[i]);
		rc_output[i] += rc_smooth * (rc_input[i] - rc_output[i]);
	}

	qc_throttle_target = (rc_output[0] - 1000.) * 0.6;
	qc_target_a[0] = (rc_output[1] - 1500.) * 0.18;                        //rc_cov = 90°/ 500 steps, 0.00628318530717958647 = 3.14159265 / 500 steps;
	qc_target_a[1] = (rc_output[2] - 1500.) * 0.18;                        //rc_cov = 90°/ 500 steps, 0.00628318530717958647 = 3.14159265 / 500 steps;
	qc_target_a[2] = (rc_output[3] - 1500.) * 0.18;                        //rc_cov = 90°/ 500 steps, 0.00628318530717958647 = 3.14159265 / 500 steps;
	
	for (int n = 0; n < 2; n++) {
		for (int i = 0; i < 4; i++) {
			if (rc_switch_val[(n * 4) + i] > 1650) {
				rc_switch_state[n] &= ~(0b00000011 << (i * 2));
			} else if (rc_switch_val[(n * 4) + i] < 1350) {
				rc_switch_state[n] &= ~(0b00000011 << (i * 2));
				rc_switch_state[n] |= 0b00000010 << (i * 2);
			} else {
				rc_switch_state[n] &= ~(0b00000011 << (i * 2));
				rc_switch_state[n] |= 0b00000001 << (i * 2);
			}
		}
	}
}

void processPilot() {
	if(((rc_switch_state[0] & 0b00000011) == 2)) {
		qc_enabled = false;
		qc_p_a.setMode(MANUAL);
		qc_error_a[1] = 0;
	}else if(((rc_switch_state[0] & 0b00000011) == 0)) {
		qc_enabled = true;
		qc_p_a.setMode(AUTOMATIC);
	}
	
	if(((rc_switch_state[0] & 0b11000000) == 128)) {
		NVIC_SystemReset();
	}
}

void parseSerial() {
	
	if(Serial.available() > 1) {
	
		comm_response = Serial.peek();
		if(comm_response == COMM_DATA) {
		
			if(Serial.available() > 7) {
				Serial.read();
				comm_checksum = 0;
		
				for(int i = 0; i < 3; i++) {
					comm_angles[i] = (uint16_t)Serial.read() << 8;
					comm_angles[i] += (uint16_t)Serial.read();
					comm_checksum = (comm_checksum + comm_angles[i]) % 255;
				}
		
				comm_message = Serial.read();
				if(comm_checksum == comm_message) {
					qc_actual_a[0] = (((float)comm_angles[0]) * INT_TO_DEG) - 180.;
					qc_actual_a[1] = (((float)comm_angles[1]) * INT_TO_DEG) - 180.;
					qc_actual_a[2] = (((float)comm_angles[2]) * INT_TO_DEG) - 180.;
				}
			}
		
		}else if(comm_response == COMM_DATA_2) {
		
			if(Serial.available() > 13) {
				Serial.read();
				long buff = 0;
				float val[3] = {0};
				comm_checksum = 0;
				
				for(int i = 0; i < 3; i++) {
					buff = 0;
					for (int c = 0; c < 4; c++) {
						buff += (Serial.read() << c*8);
					}
					memcpy(&val[i], &buff, 4);
					comm_checksum += abs((int)val[i]);
				}
				comm_message = Serial.read();
				if((comm_checksum % 255) == comm_message) {
					qc_actual_r[0] = val[0];
					qc_actual_r[1] = val[1];
					qc_actual_r[2] = val[2];
				}
			}
		
		}else if(comm_response == COMM_INFO) {
		
			if(Serial.available() > 1) {
				
				Serial.read();
				if(Serial.peek() == COMM_END) {
				
					Serial.read();
					Serial.write(COMM_END);
					Serial.print("Restarting\n");
					delay(1);
					NVIC_SystemReset();
				
				}else if(Serial.peek() == COMM_FAIL) {
			
					Serial.read();
					Serial.write(COMM_FAIL);
					Serial.print("Host fail\n");
					//NVIC_SystemReset();
				}
			}
		}else {
			for(int n = 0; n < 10; n++) {
				if(Serial.available() > 0) {
					comm_response = Serial.peek();
					if(comm_response == COMM_DATA || comm_response == COMM_INFO) {
						break;
					}else {
						Serial.read();
					}
				}
			}
		}
	}
}

void updateESCs() {
	if(qc_enabled) {
		qc_esc_us[0] = 1050 + qc_throttle_target - qc_error_a[1];
		qc_esc_us[2] = 1050 + qc_throttle_target + qc_error_a[1];
		if(qc_esc_us[0] < 1000) {
			qc_esc_us[0] = 1000;
		}else if(qc_esc_us[0] > 1500) {
			qc_esc_us[0] = 1500;
		}
		if(qc_esc_us[2] < 1000) {
			qc_esc_us[2] = 1000;
		}else if(qc_esc_us[2] > 1500) {
			qc_esc_us[2] = 1500;
		}
	}else{
		qc_esc_us[0] = 0;
		qc_esc_us[2] = 0;
	}
}

void writeESCs() {
	
	ESCs[0].writeMicroseconds(qc_esc_us[0]);
	ESCs[2].writeMicroseconds(qc_esc_us[2]);
	
}

void throttleHandler() {
	//if(REG_PORT_IN0 & PORT_PA18) {
	if (rc_start_time[0] == 0) {
		rc_start_time[0] = micros();
	} else {
		rc_val_last[0] = rc_val[0];
		rc_val[0] = (int)(3000L - (micros() - rc_start_time[0]));
		rc_start_time[0] = 0;
	}
}

void rollHandler() {
	//if(REG_PORT_IN0 & PORT_PA06) {
	if (rc_start_time[2] == 0) {
	rc_start_time[2] = micros();
	} else {
	rc_val_last[2] = rc_val[2];
	rc_val[2] = (int)(3000L - (micros() - rc_start_time[2]));
	rc_start_time[2] = 0;
	}
}

void pitchHandler() {
	//if(REG_PORT_IN0 & PORT_PA07) {
	if (rc_start_time[1] == 0) {
	rc_start_time[1] = micros();
	} else {
	rc_val_last[1] = rc_val[1];
	rc_val[1] = (int)(3000L - (micros() - rc_start_time[1]));
	rc_start_time[1] = 0;
	}
}

void yawHandler() {
	//if(REG_PORT_IN0 & PORT_PA16) {
	if (rc_start_time[3] == 0) {
		rc_start_time[3] = micros();
	} else {
		rc_val_last[3] = rc_val[3];
		rc_val[3] = (int)(3000L - (micros() - rc_start_time[3]));
		rc_start_time[3] = 0;
	}
}

void functionHandler() {
	rc_now = micros();
	rc_val[4] = rc_now - rc_start_time[4];
	rc_start_time[4] = rc_now;
	if (rc_val[4] < 1000) {
		rc_index = -1;
	} else if (rc_val[4] >= 15000) {
		rc_index = rc_index+1 > 6 ? 7 : rc_index+1;
	} else {
		rc_switch_val[rc_index] = rc_val[4];
	}
}
