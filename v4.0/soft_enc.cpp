#include "soft_enc.h"
#include "hFramework.h"

void handler1(void* p)
{
	soft_enc_desc* desc = (soft_enc_desc*)p;

	switch (desc->encPins) {
	case 0x00:
		desc->enc_cnt--;
		break;
	case 0x01:
		desc->enc_cnt++;
		break;
	case 0x02:
		desc->enc_cnt++;
		break;
	case 0x03:
		desc->enc_cnt--;
		break;
	default:
		break;
	};

	if (desc->pin1->read()) {
		desc->encPins |= 0x01;
	} else {
		desc->encPins &= 0xfe;
	}
}

void handler2(void* p)
{
	soft_enc_desc* desc = (soft_enc_desc*)p;

	switch (desc->encPins) {
	case 0x00:
		desc->enc_cnt++;
		break;
	case 0x01:
		desc->enc_cnt--;
		break;
	case 0x02:
		desc->enc_cnt--;
		break;
	case 0x03:
		desc->enc_cnt++;
		break;
	default:
		break;
	};

	if (desc->pin2->read()) {
		desc->encPins |= 0x02;
	} else {
		desc->encPins &= 0xfd;
	}
}

void soft_enc::init()
{
	desc.enc_cnt = 0;
	desc.encPins |= ((desc.pin2->read()) << 1) + (desc.pin1->read());
	
	desc.pin1->interruptOn_EdgeChange(CountingMode::Normal, handler1, &desc);
	desc.pin2->interruptOn_EdgeChange(CountingMode::Normal, handler2, &desc);
	
	desc.enc_cnt = 0;
}

soft_enc::soft_enc(IGPIO_int& p1, IGPIO_int& p2)
{
	desc.pin1 = &p1;
	desc.pin2 = &p2;
}

int32_t soft_enc::getEncoderCnt()
{
	return desc.enc_cnt;
}

void soft_enc::resetEncoderCnt()
{
	desc.enc_cnt = 0;
}