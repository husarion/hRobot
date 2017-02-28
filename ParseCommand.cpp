// hRobot_0_05_09 edited by mzabinski94@gmail.com
// from hRobot_0_05_06

#include <cstddef>
#include <cstdint>
#include <stdio.h>
#include <string.h>

#include "hFramework.h"
#include "ParseCommand.h"
#include "MotionManager.h"
#include "ErrorLog.h"
#include "Addons.h"
#include "UI_Buttons.h"

ParseCommand parseCommand(Serial);

void clearChar(char* t, int size)
{
	for (int i = 0; i < size; i++)
		t[i] = 0;
}

void ComandInputTask()
{
	char command[20];
	char param1[20];
	char param2[20];
	char param3[20];
	char param4[20];
	char param5[20];
	char param6[20];
	char param7[20];

	sys.setSysLogDev(&devNull);

	for (;;) {
		sys.delay(100);

		if (1 == parseCommand.parse(command, param1, param2, param3, param4, param5, param6, param7)) {
			//printf("%s\n", command);
			if (strcmp(command, "MOVE") == 0) {
				ErrorLogs::Err().send(MotionManager::get().Move(jointsInter, param1));
			}
			if (strcmp(command, "MOVES") == 0) {
				ErrorLogs::Err().send(MotionManager::get().Move(cartesianInter, param1));
			}
			if (strcmp(command, "SET") == 0) {
				if (strcmp(param2, "J") == 0) {
					MotionManager::get().addPoint(param1, jointsCo, atof(param3), atof(param4), atof(param5), atof(param6), atof(param7));
				}
				if (strcmp(param2, "R") == 0) {
					MotionManager::get().addPoint(param1, cylindricalCo, atof(param3), atof(param4), atof(param5), atof(param6), atof(param7));
				}
				if (strcmp(param2, "C") == 0) {
					MotionManager::get().addPoint(param1, cartesianCo, atof(param3), atof(param4), atof(param5), atof(param6), atof(param7));
				}
				if (strcmp(param2, "HERE") == 0) {
					//twoży punkt o nazwie param1 z obecnej pozycji
					if (strcmp(param3, "J") == 0) {
						MotionManager::get().addPoint(param1, jointsCo);
					}
					if (strcmp(param3, "R") == 0) {
						MotionManager::get().addPoint(param1, cylindricalCo);
					}
					if (strcmp(param3, "C") == 0) {
						MotionManager::get().addPoint(param1, cartesianCo);
					}
				}
			}
			if (strcmp(command, "SHOWALL") == 0) {
			    MotionManager::get().showAll();
			}
			if (strcmp(command, "SHOWCURRENT") == 0) {
			    MotionManager::get().showCurrent();
			}
			if (strcmp(command, "SHOW") == 0) {
				if (strcmp(param2, "") == 0) {
					MotionManager::get().show(param1);
				}
				if (strcmp(param2, "J") == 0) {
					MotionManager::get().show(param1, jointsCo);
				}
				if (strcmp(param2, "R") == 0) {
					MotionManager::get().show(param1, cylindricalCo);
				}
				if (strcmp(param2, "C") == 0) {
					MotionManager::get().show(param1, cartesianCo);
				}
			}
			if (strcmp(command, "DELAY") == 0){
				MotionManager::get().addMotionInst(atof(param1), 0, 0, 0, 0, Delay);
			}
			if (strcmp(command, "PRECYSION") == 0){
				if (strcmp(param1, "ON") == 0){
					MotionManager::get().setPrecysionMode(true, atoi(param2));
					ErrorLogs::Err().sendPar(28, atoi(param2));
				}
				if (strcmp(param1, "OFF") == 0){
					MotionManager::get().setPrecysionMode(false, atoi(param2));
					ErrorLogs::Err().sendPar(29, atoi(param2));
				}
			}
			if (strcmp(command, "CONFIG") == 0) {
				if (strcmp(param1, "COM") == 0) {
					if (strcmp(param2, "UI") == 0) {
						parseCommand.changeToUI();
						ErrorLogs::Err().send(21);
					}
					if (strcmp(param2, "SERIAL") == 0) {
						parseCommand.changeToSerial();
						ErrorLogs::Err().send(22);
					}
				}
				if (strcmp(param1, "OFFSET") == 0) {
					if (strcmp(param2, "ONPOINT") == 0) {
						MotionManager::get().setOffset();
					}
					if (strcmp(param2, "INPOINT") == 0) {
						MotionManager::get().setOffset(param3);
					}
				}
				if (strcmp(param1, "RESETPOINTS") == 0) {
					MotionManager::get().clearPoints();
				}
			}
            
            if (strcmp(command, "H1OPEN") == 0) {
			    MotionManager::get().GriperOpen();
			}
			if (strcmp(command, "H1CLOSE") == 0) {
			    MotionManager::get().GriperClose();
			}
			if (strcmp(command, "H1STOP") == 0) {
			    MotionManager::get().GriperStop();
			}
            
			clearChar(command, 20);
			clearChar(param1, 20);
			clearChar(param2, 20);
			clearChar(param3, 20);
			clearChar(param4, 20);
			clearChar(param5, 20);
			clearChar(param6, 20);
			clearChar(param7, 20);
		}
	}
}

ParseCommand::ParseCommand(hStreamDev & dev)
{
	d = &dev;
	state = PARSE_END;

	i_c = 0;
	i_1 = 0;
	i_2 = 0;
	i_3 = 0;
	i_4 = 0;
	i_5 = 0;
	i_6 = 0;
	i_7 = 0;

	UIconection = false;
}

void changeInputToUI()
{
	parseCommand.changeToUI();
}

void changeInputToSerial()
{
	parseCommand.changeToSerial();
}

void ParseCommand::changeToUI()
{
	UIconection = true;
}

void ParseCommand::changeToSerial()
{
	UIconection = false;
}

void ParseCommand::setStream(hStreamDev & dev)
{
	d = &dev;
	state = PARSE_END;

	i_c = 0;
	i_1 = 0;
	i_2 = 0;
	i_3 = 0;
	i_4 = 0;
	i_5 = 0;
	i_6 = 0;
	i_7 = 0;
}

bool ParseCommand::parse(char* command, char* p1, char* p2, char* p3, char* p4, char* p5, char* p6, char* p7)
{
	char c;
	char begin_str[] = "\r\n> ";

	if (state == PARSE_END) {
		d->write(begin_str, sizeof(begin_str));
		state = PARSE_COMMAND;
	}

	if (UIconection) {
		readUI(&c, 1);
	} else {
		d->read(&c, 1);
	}
	//LED2.toggle();

	if (c == '\r') {
		return 0;
	}

	if ((c == '\n')/* || (c == 'r')*/) {
		i_c = 0;
		i_1 = 0;
		i_2 = 0;
		i_3 = 0;
		i_4 = 0;
		i_5 = 0;
		i_6 = 0;
		i_7 = 0;
		state = PARSE_END;
		return 1;
	}

	if (state == PARSE_ARG7) {
		if (c == ' ') {
			*(p7 + i_7) = 0;
			state = PARSE_WAIT_ENTER;
			i_7 = 0;
		} else {
			*(p7 + i_7) = c;
			*(p7 + i_7 + 1) = 0;
			i_7++;
		}
	}

	if (state == PARSE_ARG6) {
		if (c == ' ') {
			*(p6 + i_6) = 0;
			state = PARSE_ARG7;
			i_6 = 0;
		} else {
			*(p6 + i_6) = c;
			*(p6 + i_6 + 1) = 0;
			i_6++;
		}
	}

	if (state == PARSE_ARG5) {
		if (c == ' ') {
			*(p5 + i_5) = 0;
			state = PARSE_ARG6;
			i_5 = 0;
		} else {
			*(p5 + i_5) = c;
			*(p5 + i_5 + 1) = 0;
			i_5++;
		}
	}

	if (state == PARSE_ARG4) {
		if (c == ' ') {
			*(p4 + i_4) = 0;
			state = PARSE_ARG5;
			i_4 = 0;
		} else {
			*(p4 + i_4) = c;
			*(p4 + i_4 + 1) = 0;
			i_4++;
		}
	}

	if (state == PARSE_ARG3) {
		if (c == ' ') {
			*(p3 + i_3) = 0;
			state = PARSE_ARG4;
			i_3 = 0;
		} else {
			*(p3 + i_3) = c;
			*(p3 + i_3 + 1) = 0;
			i_3++;
		}
	}

	if (state == PARSE_ARG2) {
		if (c == ' ') {
			*(p2 + i_2) = 0;
			state = PARSE_ARG3;
			i_2 = 0;
		} else {
			*(p2 + i_2) = c;
			*(p2 + i_2 + 1) = 0;
			i_2++;
		}
	}

	if (state == PARSE_ARG1) {
		if (c == ' ') {
			*(p1 + i_1) = 0;
			state = PARSE_ARG2;
			i_1 = 0;
		} else {
			*(p1 + i_1) = c;
			*(p1 + i_1 + 1) = 0;
			i_1++;
		}
	}

	if (state == PARSE_COMMAND) {
		if (c == ' ') {
			*(command + i_c) = 0;
			state = PARSE_ARG1;
			i_c = 0;
		} else {
			*(command + i_c) = c;
			*(command + i_c + 1) = 0;
			i_c++;
		}
	}

	return 0;
}
