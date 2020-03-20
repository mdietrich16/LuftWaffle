/* 
 * File:   Logger.hpp
 * Author: TheBeast
 *
 * Created on 13. März 2016, 21:31
 */

#ifndef LOGGER_HPP
#define	LOGGER_HPP

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <ctime>

using namespace std;

typedef enum {
	DEBUG = 0,
	INFO = 1,
	WARNING = 2,
	ERROR = 3,
	CRITICAL = 4,
	DISABLED = 10
} logger_level;

class Logger {
public:
	static int init(logger_level level);
	static int init4cal();
	static int log(logger_level level, string format, ...);
	static int log4cal(string format, ...);
	static int flush();
	static int close();
	static int close4cal();
private:
	//static ofstream *p_file;
	static string buffer;
	static FILE* p_file;
	static logger_level e_level;
	static bool b_initialized;
	static time_t t_starttime;
	static time_t t_ufnow;
	static tm *t_now;
	static FILE* p_file_cal;
	static bool b_initialized_cal;
	static time_t t_starttime_cal;
	static time_t t_ufnow_cal;
	static tm *t_now_cal;
};

#endif	/* LOGGER_HPP */

