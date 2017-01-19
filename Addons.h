#include <cstddef>
#include <cstdint>
#include "hFramework.h"

#ifndef __Addons__
#define __Addons__

float saturateFloat(float val, float bord);
float saturateFloatUnsym(float val, float max, float min);
float thresholdFloat(float val, float th);
float circleFloat(float val);

void printfOnConsoleInWebIDE();

#endif //__Addons__