#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"

#ifndef __UI_Buttons__
#define __UI_Buttons__

void cfgHandler();
void onButtonEvent(hId id, ButtonEventType type);
void onValueChangeEvent(hId id, const char *data);

char readUI();

#endif //__UI_Buttons__