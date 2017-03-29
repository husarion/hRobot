#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "ErrorLog.h"

void printfErrorTask()
{
    Serial.init(115200);
    for (;;)
    {
        if (ErrorLogs::err().getSize() > 0 && (int)sys.getRefTime() > 6000)
        {
            ErrorLogs::err().translateError(ErrorLogs::err().getLastError());
            sys.delay(100);
        }
        else
        {
            sys.delay(1000);
        }
    }
}

void ErrorLogs::send(int error)
{
    if (log_queue.size() <= 255)
        log_queue.push_back(error);
}

void ErrorLogs::sendPar(int error, int parametr)
{
    if (log_queue.size() <= 255)
    {
        log_queue.push_back(error);
        log_int_queue.push_back(parametr);
    }
}

void ErrorLogs::sendPar(int error, float parametr)
{
    if (log_queue.size() <= 255)
    {
        log_queue.push_back(error);
        log_float_queue.push_back(parametr);
    }
}

void ErrorLogs::sendPar(int error, char *parametr)
{
    if (log_queue.size() <= 255)
    {
        log_queue.push_back(error);
        log_char_queue.push_back(parametr);
    }
}

int ErrorLogs::getLastInt()
{
    int temp = 0;
    if (log_int_queue.size() > 0)
    {
        temp = log_int_queue[0];
        if (log_int_queue.size() == 1)
        {
            log_int_queue.clear();
        }
        else
        {
            log_int_queue.erase(log_int_queue.begin());
        }
    }
    return temp;
}

float ErrorLogs::getLastFloat()
{
    float temp = 0.0;
    if (log_float_queue.size() > 0)
    {
        temp = log_float_queue[0];
        if (log_float_queue.size() == 1)
        {
            log_float_queue.clear();
        }
        else
        {
            log_float_queue.erase(log_float_queue.begin());
        }
    }
    return temp;
}

void ErrorLogs::getLastChar(char *temp)
{
    if (log_char_queue.size() > 0)
    {
        temp = log_char_queue[0];
        if (log_char_queue.size() == 1)
        {
            log_char_queue.clear();
        }
        else
        {
            log_char_queue.erase(log_char_queue.begin());
        }
    }
}

int ErrorLogs::getLastError()
{
    int temp;
    if (log_queue.size() > 0)
    {
        temp = log_queue[0];
        if (log_queue.size() == 1)
        {
            log_queue.clear();
        }
        else
        {
            log_queue.erase(log_queue.begin());
        }
        return temp;
    }
    return 999;
}

int ErrorLogs::getSize()
{
    return log_queue.size();
}

void ErrorLogs::translateError(int error)
{
    char *temp;
    char *temp3;
    temp = new char[255];
    temp3 = new char[127];
    for(int i = 0; i< 255; i++)temp[i]=0;
    for(int i = 0; i< 127; i++)temp3[i]=0;
    switch (error)
    {
    case 0:
        temp = "info: OK";
        break;
    case 1:
        temp = "error: Fatal error";
        break;
    case 2:
        temp = "warning: Test 1";
        break;
    case 3:
        temp = "warning: Test 2";
        break;
    case 4:
        temp = "warning: Test 3";
        break;
    case 5:
        temp = "error: There is no error you are looking for";
        break;
    case 6:
        temp = "error: Involid joint number";
        break;
    case 7:
        temp = "info: New engine added";
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
        temp = "info: Loading configuration from memory";
        break;
    case 15:
        temp = "warning: No configuration in memory";
        break;
    case 16:
        temp = "warning: Saving configuration to memory";
        break;
    case 17:
        //platform.printf("-----------------------> Code: ", getLastInt());   //print on serial in WebIDE
        Serial.printf("-----------------------> Code: ", getLastInt());   //print on serial via USB
        temp = "\r\n";
        break;
    case 18:
        getLastChar(temp3);
        temp3[20] = '\n';
        Serial.printf("Point %s did not egsist", temp3);
        temp = "\r\n";
        break;
    case 19:
        getLastChar(temp3);
        temp3[20] = '\n';
        Serial.printf("Point of name %s already egzist", temp3);
        temp = "\r\n";
        break;
    case 20:
        getLastChar(temp3);
        temp3[20] = '\n';
        Serial.printf("No such point as : %s", temp3);
        temp = "\r\n";
        break;
    case 21:
        temp = "Jog from UI\t\n";
        break;
    case 22:
        temp = "Reding from Serial\t\n";
        break;
    case 23:
        temp = "Curent pozytion over range - stop\t\n";
        break;
    case 24:
        temp = "Set pozytion over range - stop\t\n";
        break;
    case 25:
        temp = "Set pozytion over range - stop\t\n";
        break;
    case 26:
        temp = "Moving from Serial coman line\t\n";
        break;
    case 27:
        temp = "Reading from UI\t\n";
        break;
    case 28:
        Serial.printf("Presition mode on, set time : %d", getLastInt());
        temp = "\r\n";
        break;
    case 29:
        Serial.printf("Presition mode off, set time : %d", getLastInt());
        temp = "\r\n";
        break;
    case 30:
        temp = "Comand unknown\r\n";
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