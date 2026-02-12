#include "container_bus.h"
#include <stdlib.h>
#include <stdio.h>

// 订阅者节点
typedef struct Subscriber {
    uint32_t tag_id;
    
    BusCallback cb_set; // set 回调
    BusCallback cb_get; // get 回调
    
    // 记录上一次看到的 FrameID，用于对比
    uint32_t last_frame_id;
    uint32_t last_read_count;
    
    struct Subscriber* next;
} Subscriber;

struct ContainerBus {
    Container* container; // 绑定的容器
    Subscriber* sub_head; // 订阅者链表
};

// ---------------------------------------------------------
// API 实现
// ---------------------------------------------------------

ContainerBus* container_bus_create(Container* target_container) {
    if (!target_container) return NULL;
    
    ContainerBus* bus = (ContainerBus*)malloc(sizeof(ContainerBus));
    if (bus) {
        bus->container = target_container;
        bus->sub_head = NULL;
    }
    return bus;
}

void container_bus_destroy(ContainerBus* bus) {
    if (!bus) return;
    
    Subscriber* cur = bus->sub_head;
    while (cur) {
        Subscriber* tmp = cur;
        cur = cur->next;
        free(tmp);
    }
    free(bus);
}

int container_bus_register(ContainerBus* bus, uint32_t tag_id, BusCallback cb_set, BusCallback cb_get) {
    if (!bus) return -1;
    
    // 创建新订阅节点
    Subscriber* sub = (Subscriber*)malloc(sizeof(Subscriber));
    if (!sub) return -1;
    
    sub->tag_id = tag_id;
    sub->cb_set = cb_set;
    sub->cb_get = cb_get;
    
    // 初始化状态 (尝试先获取当前状态，避免刚注册就误触发)
    ContainerStat stat;
    if (container_get_stat(bus->container, tag_id, &stat) == 0) {
        sub->last_frame_id = stat.frame_id;
        sub->last_read_count = stat.read_count;
    } else {
        // 如果该 ID 还不存在，初始化为 0
        sub->last_frame_id = 0;
        sub->last_read_count = 0;
    }
    
    // 头插法加入链表
    sub->next = bus->sub_head;
    bus->sub_head = sub;
    
    return 0;
}

void container_bus_poll(ContainerBus* bus) {
    if (!bus || !bus->container) return;
    
    Subscriber* sub = bus->sub_head;
    
    // 遍历所有订阅者
    while (sub) {
        ContainerStat stat;
        
        // 查询 Container 当前状态
        if (container_get_stat(bus->container, sub->tag_id, &stat) == 0) {
            
            // 1. 检测 SET 变化 (当前 FrameID > 上次记录的 FrameID)
            if (stat.frame_id > sub->last_frame_id) {
                sub->last_frame_id = stat.frame_id; // 更新记录
                
                // 触发回调
                if (sub->cb_set) {
                    void* data_ptr = NULL;
                    size_t len = 0;
                    // 为了回调方便，把数据指针取出来传过去
                    container_get(bus->container, sub->tag_id, &data_ptr, &len, NULL);
                    
                    sub->cb_set(sub->tag_id, data_ptr, len);
                }
            }
            
            // 2. 检测 GET 变化 (当前读次 > 上次记录的读次)
            if (stat.read_count > sub->last_read_count) {
                sub->last_read_count = stat.read_count; // 更新记录
                
                // 触发回调
                if (sub->cb_get) {
                    void* data_ptr = NULL;
                    size_t len = 0;
                    container_get(bus->container, sub->tag_id, &data_ptr, &len, NULL);
                    sub->cb_get(sub->tag_id, data_ptr, len);
                }
            }
        }
        
        sub = sub->next;
    }
}