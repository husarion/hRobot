#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "ErrorLog.h"
#include "Addons.h"

float saturateFloat(float val, float bord){ // limits float val symetricly between -bord and bord.
    if (val<-bord){ val=-bord; } else if (val>bord) val=bord;
    return val;
}
float saturateFloatUnsym(float val, float max, float min){ // limits float val unsymetricly between -min and max.
    if (val<-min){ val=-min; } else if (val>max) val=max;
    return val;
}
float thresholdFloat(float val, float th){ // cuts to zero if in threshold
    if (val>-th && val<0){ val=0; } else if (val<th && val>0) val=0;
    return val;
}
float circleFloat(float val){  // sub or add value to stay between -180 and 180
    if (val>=180){ val-=360; } else if (val<-180) val+=360;
    return val;
}

void printfOnConsoleInWebIDE()
{
    for (;;) {
		if (ErrorLogs::Err().getSize() > 0 && (int)sys.getRefTime() > 6000){
			ErrorLogs::Err().translateError(ErrorLogs::Err().getLastError());
			sys.delay(100);
		}
		else{
		sys.delay(1000);
		}
	}
}
