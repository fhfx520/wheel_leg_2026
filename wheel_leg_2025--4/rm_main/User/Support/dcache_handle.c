#include "dcache_handle.h"

#include "stm32h7xx.h"                  // Device header
#include "core_cm7.h"

#define DCACHE_LINE_SIZE    32U

void CleanDCacheForDmaTx(const void *addr, uint32_t len)
{
    uintptr_t start = (uintptr_t)addr;
    uintptr_t end   = start + len;
	
    start &= ~(DCACHE_LINE_SIZE - 1U);
    end = (end + DCACHE_LINE_SIZE - 1U)
        & ~(DCACHE_LINE_SIZE - 1U);

    SCB_CleanDCache_by_Addr(
        (uint32_t *)start,
        (int32_t)(end - start)
    );
}

void InvalidateDCacheForDmaRx(void *address, uint32_t length)
{
    uintptr_t start;
    uintptr_t end;
    uintptr_t start_aligned;
    uintptr_t end_aligned;

    /* Buffer 起始地址 */
    start = (uintptr_t)address;

    /* Buffer 结束地址之后的位置 */
    end = start + length;

    /* 起始地址向下对齐到 Cache Line */
    start_aligned = start & ~(DCACHE_LINE_SIZE - 1U);

    /* 结束地址向上对齐到 Cache Line */
    end_aligned = (end + DCACHE_LINE_SIZE - 1U)
                & ~(DCACHE_LINE_SIZE - 1U);

    /* Invalidate D-Cache */
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)start_aligned,
        (int32_t)(end_aligned - start_aligned)
    );
}
