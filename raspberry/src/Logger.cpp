/* 
 * File:   Logger.cpp
 * Author: TheBeast
 * 
 * Created on 13. März 2016, 21:31
 */

#include "Logger.hpp"

time_t	Logger::t_starttime = 0;
time_t Logger::t_ufnow = 0;
tm* Logger::t_now = 0; 
logger_level Logger::e_level = DISABLED;
bool Logger::b_initialized = false;
//ofstream* Logger::p_file = NULL;
FILE* Logger::p_file = NULL;
string Logger::buffer = "";


time_t	Logger::t_starttime_cal = 0;
time_t Logger::t_ufnow_cal = 0;
tm* Logger::t_now_cal = 0; 
bool Logger::b_initialized_cal = false;
FILE* Logger::p_file_cal = NULL;

int Logger::init(logger_level level) {
	t_starttime = time(0);
	t_now = localtime(&t_starttime);
	e_level = level;
	char filename[40];
	sprintf(filename, "%02i%02i%02i.log", (t_now->tm_year - 100), (t_now->tm_mon + 1), t_now->tm_mday);
	//static ofstream file(filename, ios::out | ios::app);
	//p_file = &file;
	p_file = fopen(filename, "a");
	b_initialized = true;
	return 0;
}

int Logger::log(logger_level level, string format, ...) {
	if(b_initialized) {
		if(level >= e_level) {
			va_list args;
			va_start(args, format);
			t_ufnow = time(0);
			t_now = localtime(&t_ufnow);
			char buff[256] = {0};
			sprintf(buff, "%02i:%02i:%02i - ", t_now->tm_hour, t_now->tm_min, t_now->tm_sec);
			buffer += buff;
			switch(level){
				case DEBUG:
					buffer += "DEBUG";
					break;
				case INFO:
					buffer += "INFO";
					break;
				case WARNING:
					buffer += "-WARNING-";
					break;
				case ERROR:
					buffer += "!ERROR!";
					break;
				case CRITICAL:
					buffer += "--!!CRITICAL!!--";
					break;
				default:
					buffer += "INFO";
			}
			buffer += ": ";
			vsprintf(buff, format.c_str(), args);
			buffer += buff;
			buffer += "\n";
			//(*buffer) << buff << endl;
		}
	}else {
		return -1;
		cout << "Logger not initialized!" << endl;
	}
	return 0;
}

int Logger::flush() {
	if(b_initialized) {
		fprintf(p_file, &buffer[0]);
		buffer = "";
	}else {
		return -1;
		cout << "Logger not initialized!" << endl;
	}
	return 0;
}

int Logger::close() {
	if(b_initialized) {
	    Logger::flush();
		if(!(fclose(p_file) == 0)) {
			cout << "Error closing the filestream" << endl;
			return -1;
		}
	}
}


int Logger::init4cal() {
	t_starttime_cal = time(0);
	t_now_cal = localtime(&t_starttime_cal);
	char filename[40];
	sprintf(filename, "%02i%02i%02i_calibration.log", (t_now_cal->tm_year - 100), (t_now_cal->tm_mon + 1), t_now_cal->tm_mday);
	p_file_cal = fopen(filename, "a");
	b_initialized_cal = true;
	return 0;
}

int Logger::log4cal(string format, ...) {
	if(b_initialized_cal) {
			va_list args;
			va_start(args, format);
			char buff[256] = {0};
			vfprintf(p_file_cal, format.c_str(), args);
			fprintf(p_file_cal, "\n");
	}else {
		return -1;
		cout << "Logger not initialized!" << endl;
	}
	return 0;
}

int Logger::close4cal(){
	if(b_initialized_cal) {
		if(!(fclose(p_file_cal) == 0)) {
			cout << "Error closing the filestream" << endl;
			return -1;
		}
	}
}