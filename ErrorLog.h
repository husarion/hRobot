#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include <iostream>
#include <cstdio>
#include <vector>

#ifndef ERRORLOG_MANIPULATOR
#define ERRORLOG_MANIPULATOR

class ErrorLogs{
private:
    std::vector <int> log_queue;
    std::vector <int> log_int_queue;
    std::vector <float> log_float_queue;
	ErrorLogs(){}
	ErrorLogs(const ErrorLogs &);
	int getLastInt();
	float getLastFloat();
public:
    static ErrorLogs & Err(){
        static ErrorLogs singleton;
        return singleton;
    }
	void send(int error);
	void sendPar(int error, int parametr);
	void sendPar(int error, float parametr);
	int getLastError();
	int getSize();
	void translateError(int error);
};

#endif