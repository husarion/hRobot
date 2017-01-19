#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "ErrorLog.h"

void ErrorLogs::send(int error){
    if(log_queue.size()<=255)
    log_queue.push_back(error);
}

void ErrorLogs::sendPar(int error, int parametr){
    if(log_queue.size()<=255){
    log_queue.push_back(error);
    log_int_queue.push_back(parametr);
    }
}

void ErrorLogs::sendPar(int error, float parametr){
    if(log_queue.size()<=255){
    log_queue.push_back(error);
    log_float_queue.push_back(parametr);
    }
}

int ErrorLogs::getLastInt(){
    int temp = 0;
    if(log_int_queue.size()>0){
        temp = log_int_queue[0];
        if(log_int_queue.size()==1){
            log_int_queue.clear();
        }
        else{
        log_int_queue.erase(log_int_queue.begin());
        }
    }
    return temp;
}

float ErrorLogs::getLastFloat(){
    float temp = 0.0;
    if(log_float_queue.size()>0){
        temp = log_float_queue[0];
        if(log_float_queue.size()==1){
            log_float_queue.clear();
        }
        else{
        log_float_queue.erase(log_float_queue.begin());
        }
    }
    return temp;
}

int ErrorLogs::getLastError(){
    int temp;
    if(log_queue.size()>0){
        temp = log_queue[0];
        if(log_queue.size()==1){
            log_queue.clear();
        }
        else{
        log_queue.erase(log_queue.begin());
        }
        return temp;
    }
    return 999;
}

int ErrorLogs::getSize(){
    return log_queue.size();
}

void ErrorLogs::translateError(int error){
    char * temp;
    temp = new char[1024];
    switch(error){
        case 0:
        temp = "info: OK";
        break;
        case 1:
        temp = "error: Fatal error";
        break;
        case 2:
        temp = "worning: Test 1";
        break;
        case 3:
        temp = "worning: Test 2";
        break;
        case 4:
        temp = "worning: Test 3";
        break;
        case 5:
        temp = "error: This is not a error you are looking for";
        break;
        case 6:
        temp = "error: Involid joint number";
        break;
        case 7:
        temp = "info: New engine add";
        break;
        case 8:
        temp = "error: Servo fault";
        break;
        case 9:
        temp = "error: Servo-encoder critical fault";
        break;
        case 10:
        temp = "info: Encoder task started";
        break;
        case 11:
        temp = "info: Motor/Servo task started";
        break;
        case 12:
        temp = "info: Controler task started";
        break;
        case 13:
        temp = "info: Client task started";
        break;
        case 14:
        temp = "info: Lading configuration from memory";
        break;
        case 15:
        temp = "worning: No configuration in memory";
        break;
        case 16:
        temp = "worning: Saving configuration to memory";
        break;
        //tu dodac wlasne bledy aby skrypt je obsluzyl
        
        //
        
        default:
        temp = "Nieznany blad";
        break;
    }
    //platform.printf("%s\n", temp);
    Serial.printf("%s\n", temp);
}