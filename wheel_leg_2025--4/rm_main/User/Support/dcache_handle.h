#pragma once

#include "stdint.h"

void CleanDCacheForDmaTx(const void *addr, uint32_t len);
void InvalidateDCacheForDmaRx(void *address, uint32_t length);
