#include "spi_log.h"
#include "spi.h"
#include "dcache_handle.h"
#include <string.h>

#if defined(__ARMCC_VERSION)
/* 缺少专用 DMA 区时必须链接失败，不能退回默认布局后静默运行。 */
extern uint8_t log_spi_dma_region_start[] __asm("Image$$RW_LOG_SPI_DMA$$Base");
#endif

typedef struct {
    uint16_t length;
    uint8_t data[LOG_SPI_MAX_PACKET];
} spi_log_slot_t;

#if !LOG_SPI_POLLING_TEST
static spi_log_slot_t queue[LOG_SPI_QUEUE_CAPACITY];
static uint32_t head, tail;
#endif
/* DMA 只访问独立且按缓存行对齐的暂存区；普通队列可由链接器放入 DTCM。 */
__attribute__((section(".bss.spi_log_dma"), aligned(32)))
static uint8_t dma_buffer[LOG_SPI_MAX_PACKET];
static uint16_t inflight_bytes;
static uint8_t initialized, config_valid;
static volatile uint8_t receiver_ready, recovery_needed;

volatile uint8_t g_spi_log_link_ready;
volatile spi_log_link_diag_t g_spi_log_link_diag;
volatile spi_tx_diag_t g_spi_tx_diag;
volatile uint32_t spi_tx_pending_count, spi_tx_inflight;
volatile uint32_t spi_tx_queue_overflow_count;

/* 调用者须屏蔽中断；HAL 启动为非阻塞操作，回调将在恢复中断后执行。 */
static void start_locked(void)
{
#if !LOG_SPI_POLLING_TEST
    HAL_StatusTypeDef result;
    uint16_t length;
    if (!g_spi_log_link_ready || recovery_needed || spi_tx_inflight ||
        spi_tx_pending_count == 0U) return;
    if (HAL_SPI_GetState(&LOG_SPI_HANDLE) != HAL_SPI_STATE_READY) return;
    length = queue[tail].length;
    memcpy(dma_buffer, queue[tail].data, length);
    CleanDCacheForDmaTx(dma_buffer, length);
    __DSB();
    inflight_bytes = length;
    spi_tx_inflight = 1U;
    result = HAL_SPI_Transmit_DMA(&LOG_SPI_HANDLE, dma_buffer, length);
    if (result == HAL_OK) {
        uint32_t i, sample_len = length < 32U ? length : 32U;
        /* 此时中断仍被屏蔽，完成回调尚不能复用 DMA 暂存区。
         * 快照记录 CPU 交给 DMA 的数据，不代表引脚波形或对端已接收。 */
        g_spi_tx_diag.last_sample_len = 0U;
        for (i = 0U; i < sample_len; i++) g_spi_tx_diag.last_sample[i] = dma_buffer[i];
        g_spi_tx_diag.last_sample_len = sample_len;
        if (g_spi_tx_diag.first_sample_len == 0U) {
            for (i = 0U; i < sample_len; i++) g_spi_tx_diag.first_sample[i] = dma_buffer[i];
            g_spi_tx_diag.first_sample_len = sample_len;
        }
        tail = (tail + 1U) % LOG_SPI_QUEUE_CAPACITY;
        spi_tx_pending_count--;
        g_spi_tx_diag.buffered = spi_tx_pending_count;
        g_spi_tx_diag.started++;
        g_spi_tx_diag.started_bytes += length;
    } else {
        /* 启动失败时仍保留队首；下一轮重试，不误报发送完成。 */
        spi_tx_inflight = 0U;
        inflight_bytes = 0U;
        g_spi_tx_diag.start_failed++;
    }
#endif
}

#if LOG_SPI_POLLING_TEST
/* 仅供独立诊断工程使用；直接从只读文本由 CPU 写入 SPI，绕过发送 DMA。
 * 只在任务中调用且不屏蔽中断，保证 HAL 超时计时和其他任务可正常运行。 */
static void polling_test_service(void)
{
    static const uint8_t packet[] = "SPI_TEST: 0123456789 ABCDEF\r\n";
    static uint32_t last_tick;
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef result;
    g_spi_log_link_ready = 0U;
    if (!receiver_ready || spi_tx_inflight ||
        HAL_SPI_GetState(&LOG_SPI_HANDLE) != HAL_SPI_STATE_READY ||
        (uint32_t)(now - last_tick) < 10U) return;
    last_tick = now;
    g_spi_tx_diag.polling_test_attempts++;
    result = HAL_SPI_Transmit(&LOG_SPI_HANDLE, packet, sizeof(packet) - 1U, 10U);
    g_spi_tx_diag.polling_test_last_status = (uint32_t)result;
    if (result == HAL_OK) g_spi_tx_diag.polling_test_completed++;
    else g_spi_tx_diag.polling_test_failed++;
}
#endif

void SPI_LogInit(void)
{
    uintptr_t address = (uintptr_t)dma_buffer;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (!initialized) {
        initialized = 1U;
        g_spi_tx_diag.polling_test_mode = LOG_SPI_POLLING_TEST;
        g_spi_tx_diag.init_count++;
        g_spi_log_link_diag.session++;
        config_valid = LOG_SPI_HANDLE.hdmatx != NULL &&
            LOG_SPI_HANDLE.Init.Mode == SPI_MODE_MASTER &&
            LOG_SPI_HANDLE.Init.Direction == SPI_DIRECTION_2LINES_TXONLY &&
            LOG_SPI_HANDLE.Init.DataSize == SPI_DATASIZE_8BIT &&
            LOG_SPI_HANDLE.hdmatx->Init.Mode == DMA_NORMAL &&
            LOG_SPI_HANDLE.hdmatx->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE &&
            LOG_SPI_HANDLE.hdmatx->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
            address >= LOG_SPI_DMA_BASE &&
            address + sizeof(dma_buffer) <= LOG_SPI_DMA_BASE + LOG_SPI_DMA_BYTES;
#if defined(__ARMCC_VERSION)
        config_valid = config_valid &&
            (uintptr_t)log_spi_dma_region_start == LOG_SPI_DMA_BASE;
#endif
    }
    __set_PRIMASK(key);
}

void SPI_LogSetReceiverReady(uint8_t ready)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (!ready && receiver_ready) {
        g_spi_log_link_ready = 0U;
        g_spi_tx_diag.link_reset_count++;
        g_spi_log_link_diag.session++;
    }
    receiver_ready = ready != 0U;
    __set_PRIMASK(key);
}

uint8_t SPI_LogTransmit(const uint8_t *data, uint16_t length)
{
    uint8_t result;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    g_spi_tx_diag.attempts++;
    if (!initialized || !config_valid || data == NULL || length == 0U ||
        length > LOG_SPI_MAX_PACKET) {
        g_spi_tx_diag.invalid++;
        result = LOG_SPI_FAIL;
    }
#if LOG_SPI_POLLING_TEST
    else {
        /* 诊断期间不混入其他任务提交的普通日志或故障回报。 */
        g_spi_tx_diag.busy++;
        result = LOG_SPI_BUSY;
    }
#else
    else if (spi_tx_pending_count == LOG_SPI_QUEUE_CAPACITY) {
        spi_tx_queue_overflow_count++;
        g_spi_tx_diag.busy++;
        result = LOG_SPI_BUSY;
    } else {
        memcpy(queue[head].data, data, length);
        queue[head].length = length;
        head = (head + 1U) % LOG_SPI_QUEUE_CAPACITY;
        spi_tx_pending_count++;
        g_spi_tx_diag.buffered = spi_tx_pending_count;
        if (spi_tx_pending_count > g_spi_tx_diag.queue_high_water)
            g_spi_tx_diag.queue_high_water = spi_tx_pending_count;
        g_spi_tx_diag.accepted++;
        g_spi_tx_diag.accepted_bytes += length;
        start_locked();
        result = LOG_SPI_OK;
    }
#endif
    __set_PRIMASK(key);
    return result;
}

void SPI_LogService(void)
{
#if !LOG_SPI_POLLING_TEST
    uint32_t key;
#endif
    if (!initialized || !config_valid) return;
#if LOG_SPI_POLLING_TEST
    polling_test_service();
#else
    if (recovery_needed) {
        /* 中止操作只在任务中执行；未知发送进度的数据不自动重发。 */
        if (HAL_SPI_Abort(&LOG_SPI_HANDLE) != HAL_OK) return;
        key = __get_PRIMASK();
        __disable_irq();
        recovery_needed = 0U;
        g_spi_tx_diag.recoveries++;
        __set_PRIMASK(key);
    }
    key = __get_PRIMASK();
    __disable_irq();
    g_spi_log_link_ready = receiver_ready && !recovery_needed;
    start_locked();
    __set_PRIMASK(key);
#endif
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint32_t key;
    if (hspi != &LOG_SPI_HANDLE) return;
    key = __get_PRIMASK();
    __disable_irq();
    if (spi_tx_inflight) {
        /* H7 的 HAL 在 EOT 结束并恢复 READY 后调用这里，才可复用暂存区。 */
        g_spi_tx_diag.completed++;
        g_spi_tx_diag.completed_bytes += inflight_bytes;
        spi_tx_inflight = 0U;
        inflight_bytes = 0U;
        start_locked();
    }
    __set_PRIMASK(key);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    uint32_t key;
    if (hspi != &LOG_SPI_HANDLE || !initialized) return;
    key = __get_PRIMASK();
    __disable_irq();
    g_spi_tx_diag.errors++;
    g_spi_tx_diag.last_error = hspi->ErrorCode;
    g_spi_tx_diag.error_lost_bytes += inflight_bytes;
    g_spi_tx_diag.link_reset_count++;
    g_spi_log_link_diag.session++;
    inflight_bytes = 0U;
    spi_tx_inflight = 0U;
    g_spi_log_link_ready = 0U;
    recovery_needed = 1U;
    __set_PRIMASK(key);
}
