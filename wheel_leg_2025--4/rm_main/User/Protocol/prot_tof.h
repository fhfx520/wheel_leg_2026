#ifndef __PROT_TOF_H
#define __PROT_TOF_H

#include "stdint.h"


#define TOF_HEAD	0x20
#define TOF_TAIL	0x0A
typedef struct
{
	uint16_t dis;//mm
	uint8_t confidence;//÷√–≈∂»0-100
} tof_t;


extern tof_t tof[2];
void tof_get_data(uint8_t *buff, uint16_t len, uint8_t side);

#endif
