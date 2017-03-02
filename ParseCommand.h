#ifndef __PARSE_COMMAND__
#define __PARSE_COMMAND__

#include "hTypes.h"
#include <IGPIO.h>
#include <hStreamDev.h>

using namespace hFramework;

void changeInputToUI();
void changeInputToSerial();

void ComandInputTaskSerial();
void ComandInputTaskUI();

typedef enum {
	PARSE_COMMAND,
	PARSE_ARG1,
	PARSE_ARG2,
	PARSE_ARG3,
	PARSE_ARG4,
	PARSE_ARG5,
	PARSE_ARG6,
	PARSE_ARG7,
	PARSE_WAIT_ENTER,
	PARSE_END
} ParserState;

class ParseCommand {
public:
	~ParseCommand() { }
	ParseCommand(hStreamDev& dev, bool fromUI);
	ParseCommand(bool fromUI);
	void setStream(hStreamDev& dev);
	bool UIconection;
	bool UIconectionStatic;
	void changeToUI();
	void changeToSerial();
	void ParseCommandActiveSet(bool b);
	
	bool parse(char* command, char* p1, char* p2, char* p3, char* p4, char* p5, char* p6, char* p7);
private:
    hStreamDev* d;
    ParserState state;
    
    uint32_t i_c;
	uint32_t i_1;
	uint32_t i_2;
	uint32_t i_3;
	uint32_t i_4;
	uint32_t i_5;
	uint32_t i_6;
	uint32_t i_7;
};

#endif //__PARSE_COMMAND__