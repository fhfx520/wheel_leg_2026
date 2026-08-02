#include "prot_tof.h"
#include <stddef.h>
#include "stm32h7xx.h"                  // Device header


tof_t tof[2];
void tof_get_data(uint8_t *buff, uint16_t len, uint8_t side)
{
	uint16_t distance = 0;
	uint8_t confidence = 0;
	uint8_t i = 1;
	if(len < 6 || len > 11 || buff == NULL)
		return;
	if(buff[0] == TOF_HEAD)
	{
		for(;buff[i] != 0x2C;i++)
		{
			uint16_t next_distance = (uint16_t)(distance * 10u + (uint16_t)(buff[i] - '0'));
			if(next_distance < distance)
				return;
			distance = next_distance;
		}
		i += 2;
		for(;buff[i] != TOF_TAIL;i++)
		{
			uint8_t next_confidence = (uint8_t)(confidence * 10u + (uint8_t)(buff[i] - '0'));
			if(next_confidence < confidence)
				return;
			confidence = next_confidence;
		}
		tof[side].dis = distance;
		tof[side].confidence = confidence;
		tof[side].last_rx_tick = HAL_GetTick();
		return;
	}
}

void tof_check_offline(void)
{
	uint32_t current_tick = HAL_GetTick();
	for(uint8_t side = 0;side < 2;side++)
	{
		if(tof[side].last_rx_tick != 0 && (uint32_t)(current_tick - tof[side].last_rx_tick) <= TOF_MAX_OFFLINE_TIMEOUT_MS)
		{
			tof[side].online = 1;
		}
		else
		{
			tof[side].online = 0;
		}
	}
}

