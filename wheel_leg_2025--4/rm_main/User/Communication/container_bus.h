#ifndef _CONTAINER_BUS_H_
#define _CONTAINER_BUS_H_

#include "container.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ==================================================================================
 * BUS USAGE EXAMPLE
 * ==================================================================================
 *
 * // 1. 定义回调
 * void on_imu_data(uint32_t tag_id, void* data, size_t len) {
 * // 处理 IMU 数据...
 * }
 *
 * // 2. 初始化
 * Container* c = container_create();
 * ContainerBus* bus = container_bus_create(c);
 *
 * // 3. 注册监听 (监听 set 动作)
 * container_bus_register(bus, 0x10, on_imu_data, NULL);
 *
 * // 4. 主循环调用
 * while(1) {
 * container_bus_poll(bus); // 轮询检查 frame_id 变化
 * HAL_Delay(5);
 * }
 * ==================================================================================
 */

// 回调函数定义
typedef void (*BusCallback)(uint32_t tag_id, void* data, size_t len);

typedef struct ContainerBus ContainerBus;

/**
 * @brief 创建 Bus 并绑定到一个 Container 实例
 */
ContainerBus* container_bus_create(Container* target_container);

/**
 * @brief 销毁 Bus
 */
void container_bus_destroy(ContainerBus* bus);

/**
 * @brief 注册监听器
 * @param bus 句柄
 * @param tag_id 要监听的 ID
 * @param cb_set 当有人调用 container_set (frame_id 增加) 时触发
 * @param cb_get 当有人调用 container_get (read_count 增加) 时触发
 * @return 0 成功
 */
int container_bus_register(ContainerBus* bus, uint32_t tag_id, BusCallback cb_set, BusCallback cb_get);

/**
 * @brief 轮询任务
 * @note 检查 ContainerStat 中的 frame_id 是否改变，改变则触发回调
 */
void container_bus_poll(ContainerBus* bus);

#ifdef __cplusplus
}
#endif

#endif // _CONTAINER_BUS_H_