#ifndef _CONTAINER_H_
#define _CONTAINER_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// --- FreeRTOS 集成 ---
#include "FreeRTOS.h"
#include "semphr.h"
// --------------------

#ifdef __cplusplus
extern "C" {
#endif

// 支持的数据类型
typedef enum {
    CONTAINER_TYPE_UNKNOWN = 0,
    CONTAINER_TYPE_INT,
    CONTAINER_TYPE_FLOAT,
    CONTAINER_TYPE_STRING,
    CONTAINER_TYPE_STRUCT,
    CONTAINER_TYPE_RAW
} ContainerDataType;

// 状态结构体
typedef struct {
    uint32_t frame_id;   
    uint32_t read_count; 
    size_t length;       
} ContainerStat;

typedef struct Container Container;

/**
 * @brief 创建一个新的 Container 实例 (包含 Mutex 初始化)
 */
Container* container_create(void);

/**
 * @brief 销毁 Container
 */
void container_destroy(Container* ctx);

/**
 * @brief (线程安全) 存入数据
 * @note 内部会获取 Mutex，阻塞直到获取成功
 */
int container_set(Container* ctx, uint32_t tag_id, const void* data, size_t data_len, ContainerDataType type);

/**
 * @brief (线程安全) 获取数据
 */
int container_get(Container* ctx, uint32_t tag_id, void** out_data, size_t* out_len, ContainerDataType* out_type);

/**
 * @brief (线程安全) 获取状态
 */
int container_get_stat(Container* ctx, uint32_t tag_id, ContainerStat* out_stat);

void container_dump_info(Container* ctx);

#ifdef __cplusplus
}
#endif

#endif // _CONTAINER_H_