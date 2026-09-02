#pragma once

#include "stdint.h"

#define DCACHE_LINE_SIZE    32U

void CleanDCacheForDmaTx(const void *addr, uint32_t len);
void InvalidateDCacheForDmaRx(void *address, uint32_t length);
