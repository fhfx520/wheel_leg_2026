#ifndef _CONTAINER_H_
#define _CONTAINER_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ==================================================================================
 * USAGE EXAMPLE (STM32 Compatible)
 * ==================================================================================
 *
 * void example_usage() {
 * // 1. 创建容器 (Lazy Init: 此时不分配大内存)
 * Container* box = container_create();
 * if (!box) {
 * // Handle error (heap full?)
 * return;
 * }
 *
 * // 2. 准备数据
 * int config_id = 1001;
 * float voltage = 3.3f;
 * char status = 'A'; // 1 byte
 *
 * // 3. 存入数据 (set)
 * // 内部会自动处理 4 字节对齐，防止 STM32 HardFault
 * container_set(box, 0x01, &config_id, sizeof(int), CONTAINER_TYPE_INT);
 * container_set(box, 0x02, &status, sizeof(char), CONTAINER_TYPE_RAW); 
 * container_set(box, 0x03, &voltage, sizeof(float), CONTAINER_TYPE_FLOAT);
 *
 * // 4. 获取数据 (get - Zero Copy)
 * void* ptr = NULL;
 * size_t len = 0;
 * ContainerDataType type;
 *
 * // 获取 ID 0x03 (float)
 * if (container_get(box, 0x03, &ptr, &len, &type) == 0) {
 * // 因为做了对齐，这里强转 float* 是安全的
 * float val = *(float*)ptr; 
 * }
 *
 * // 5. 调试查看 (通过串口打印)
 * container_dump_info(box);
 *
 * // 6. 销毁
 * container_destroy(box);
 * }
 *
 * ==================================================================================
 */

// 支持的数据类型枚举
typedef enum {
    CONTAINER_TYPE_UNKNOWN = 0,
    CONTAINER_TYPE_INT,
    CONTAINER_TYPE_FLOAT,
    CONTAINER_TYPE_STRING,
    CONTAINER_TYPE_STRUCT,
    CONTAINER_TYPE_RAW
} ContainerDataType;

// 前向声明，对用户隐藏具体实现细节
typedef struct Container Container;

/**
 * @brief 创建一个新的 Container 实例
 * @note 采用惰性初始化，创建时不分配大内存，首次 set 时分配
 * @return Container* 句柄，失败返回 NULL
 */
Container* container_create(void);

/**
 * @brief 销毁 Container 并释放所有资源
 */
void container_destroy(Container* ctx);

/**
 * @brief 向 Container 中存入数据
 * @note 1. 如果 tag_id 已存在，新数据会追加到 Buffer 末尾，并更新索引指向新位置
 * 2. 自动执行 4 字节对齐，确保 STM32 访问安全
 * @param ctx 句柄
 * @param tag_id 数据的唯一标识 ID
 * @param data 要存储的数据指针
 * @param data_len 数据长度 (字节)
 * @param type 数据类型（用于元数据记录）
 * @return int 0 表示成功，-1 表示失败
 */
int container_set(Container* ctx, uint32_t tag_id, const void* data, size_t data_len, ContainerDataType type);

/**
 * @brief 从 Container 中获取数据
 * @param ctx 句柄
 * @param tag_id 要获取的 ID
 * @param out_data 输出参数：指向内部缓冲区的指针 (零拷贝)
 * @param out_len 输出参数：数据的长度
 * @param out_type 输出参数：数据的类型
 * @return int 0 表示找到，-1 表示未找到
 */
int container_get(Container* ctx, uint32_t tag_id, void** out_data, size_t* out_len, ContainerDataType* out_type);

/**
 * @brief 打印当前 Container 的状态（调试用，依赖 printf）
 */
void container_dump_info(Container* ctx);

#ifdef __cplusplus
}
#endif

#endif // _CONTAINER_H_