#ifndef __SOFT_ENCODER__
#define __SOFT_ENCODER__

#include "hTypes.h"
#include <IGPIO.h>

using namespace hFramework;
using namespace interfaces;

struct soft_enc_desc
{
    IGPIO_int *pin1;
    IGPIO_int *pin2;
    int32_t enc_cnt;
    uint8_t encPins;
};

class soft_enc
{
  public:
    ~soft_enc() {}
    soft_enc(IGPIO_int &p1, IGPIO_int &p2);
    void init();
    /**
	 * @brief Get number of encoder ticks.
	 * @return encoder ticks
	 */
    int32_t getEncoderCnt();

    /**
	 * @brief Set encoder ticks to 0
	 */
    void resetEncoderCnt();

  private:
    soft_enc_desc desc;
};

#endif //__SOFT_ENCODER__