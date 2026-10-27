#ifndef SPI_LOG_CONFIG_H
#define SPI_LOG_CONFIG_H

/* SPI 专供日志使用，不得与灯带或其他驱动共用同一个句柄。 */
#define LOG_SPI_HANDLE hspi1
#define LOG_SPI_QUEUE_CAPACITY 32U
#define LOG_SPI_MAX_PACKET 768U

/* 独立诊断工程设为 1：用 CPU 发送固定文本，暂停普通日志和故障补报。
 * 正常工程保持为 0，继续使用原来的 DMA 队列发送。 */
#ifndef LOG_SPI_POLLING_TEST
#define LOG_SPI_POLLING_TEST 0U
#endif

/* 与链接脚本保持一致，DMA 暂存区必须位于 H723 的 AXI SRAM。 */
#define LOG_SPI_DMA_BASE 0x2404E000
#define LOG_SPI_DMA_BYTES 0x00001000

#endif
