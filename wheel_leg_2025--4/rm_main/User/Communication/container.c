#include "container.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// 默认初始分配大小 (例如 512 字节，适应 STM32 小内存)
#define INITIAL_CAPACITY 512

// 定义对齐字节数 (STM32 Cortex-M 通常需要 4 字节对齐)
#define ALIGN_SIZE 4

// 辅助宏：计算向上对齐后的数值
// 例如：ALIGN_UP(1) -> 4, ALIGN_UP(4) -> 4, ALIGN_UP(5) -> 8
#define ALIGN_UP(size) (((size) + (ALIGN_SIZE - 1)) & ~(ALIGN_SIZE - 1))

// ---------------------------------------------------------
// 内部结构定义
// ---------------------------------------------------------

// 链表节点：只存储元数据 (Metadata)
typedef struct Node {
    uint32_t tag_id;        // Key
    ContainerDataType type; // Type
    size_t offset;          // Value 在 buffer 中的偏移量
    size_t length;          // Value 的长度
    struct Node* next;      // 链表指针
} Node;

// Container 管理结构体
struct Container {
    Node* head;             // 索引链表头
    uint8_t* buffer;        // 数据堆 (Heap/Pool)
    size_t capacity;        // 总容量
    size_t used_size;       // 当前已使用
};

// ---------------------------------------------------------
// 内部辅助函数
// ---------------------------------------------------------

static Node* find_node(Container* ctx, uint32_t tag_id) {
    Node* current = ctx->head;
    while (current != NULL) {
        if (current->tag_id == tag_id) {
            return current;
        }
        current = current->next;
    }
    return NULL;
}

// 智能扩容逻辑
static int ensure_capacity(Container* ctx, size_t required_len) {
    size_t needed_total = ctx->used_size + required_len;

    // 1. 首次使用初始化
    if (ctx->buffer == NULL) {
        size_t start_size = (required_len > INITIAL_CAPACITY) ? required_len : INITIAL_CAPACITY;
        
        // 确保初始分配也是对齐的（虽然 malloc 通常是对齐的，但习惯上保持一致）
        start_size = ALIGN_UP(start_size);
        
        ctx->buffer = (uint8_t*)malloc(start_size);
        if (!ctx->buffer) return -1;
        ctx->capacity = start_size;
        return 0;
    }

    // 2. 空间不足扩容
    if (needed_total > ctx->capacity) {
        size_t new_capacity = ctx->capacity;
        
        // 指数级增长：不够就翻倍，直到够为止
        while (new_capacity < needed_total) {
            new_capacity *= 2; 
        }

        // realloc 可能会移动内存块，但 offset 保持相对不变，安全
        uint8_t* new_buffer = (uint8_t*)realloc(ctx->buffer, new_capacity);
        if (!new_buffer) {
            return -1; // 内存不足
        }

        ctx->buffer = new_buffer;
        ctx->capacity = new_capacity;
    }
    return 0;
}

// ---------------------------------------------------------
// API 实现
// ---------------------------------------------------------

Container* container_create(void) {
    Container* ctx = (Container*)malloc(sizeof(Container));
    if (!ctx) return NULL;

    // 初始化置空，等待第一次 set 时分配内存
    ctx->buffer = NULL;
    ctx->capacity = 0;
    ctx->used_size = 0;
    ctx->head = NULL;

    return ctx;
}

void container_destroy(Container* ctx) {
    if (!ctx) return;

    // 1. 释放链表节点
    Node* current = ctx->head;
    while (current != NULL) {
        Node* temp = current;
        current = current->next;
        free(temp);
    }

    // 2. 释放大缓冲区
    if (ctx->buffer) {
        free(ctx->buffer);
    }

    // 3. 释放管理结构
    free(ctx);
}

int container_set(Container* ctx, uint32_t tag_id, const void* data, size_t data_len, ContainerDataType type) {
    if (!ctx || !data || data_len == 0) return -1;

    // STM32 关键步骤：计算对齐后的写入位置
    // 我们希望数据的起始地址 (buffer + offset) 总是 4 的倍数
    // 这样用户在 get 时强转 (int*) 或 (struct*) 才不会 HardFault
    size_t current_offset = ctx->used_size;
    size_t aligned_offset = ALIGN_UP(current_offset);
    
    // 计算需要的增量空间：(对齐填充 padding) + (实际数据长度)
    size_t padding = aligned_offset - current_offset;
    size_t total_needed = padding + data_len;

    // 检查并扩容
    if (ensure_capacity(ctx, total_needed) != 0) {
        return -1; 
    }

    Node* node = find_node(ctx, tag_id);

    // 写入数据 (从对齐后的位置开始写)
    // 注意：ctx->used_size 到 aligned_offset 之间的内存是 Padding，保持原样即可
    memcpy(ctx->buffer + aligned_offset, data, data_len);
    
    // 更新已用大小
    ctx->used_size = aligned_offset + data_len;

    if (node) {
        // 更新现有节点的索引信息
        node->offset = aligned_offset;
        node->length = data_len;
        node->type = type;
    } else {
        // 创建新节点 (头插法，效率高)
        Node* new_node = (Node*)malloc(sizeof(Node));
        if (!new_node) return -1;

        new_node->tag_id = tag_id;
        new_node->type = type;
        new_node->offset = aligned_offset;
        new_node->length = data_len;
        
        new_node->next = ctx->head;
        ctx->head = new_node;
    }

    return 0;
}

int container_get(Container* ctx, uint32_t tag_id, void** out_data, size_t* out_len, ContainerDataType* out_type) {
    if (!ctx || !out_data) return -1;

    Node* node = find_node(ctx, tag_id);
    if (!node) {
        return -1; // Not found
    }

    // 返回 buffer 中的直接指针 (零拷贝)
    // 由于 set 时做了对齐，这里返回的地址在 STM32 上是安全的
    *out_data = (ctx->buffer + node->offset);
    
    if (out_len) *out_len = node->length;
    if (out_type) *out_type = node->type;

    return 0;
}

void container_dump_info(Container* ctx) {
    if (!ctx) return;
    printf("\n--- Container Dump ---\n");
    printf("Capacity : %zu bytes\n", ctx->capacity);
    printf("Used     : %zu bytes\n", ctx->used_size);
    printf("Base Addr: %p\n", (void*)ctx->buffer);
    
    Node* cur = ctx->head;
    int count = 0;
    while(cur) {
        printf("  [Node %d] Tag: 0x%02X, Offset: %zu, Len: %zu", 
               count++, cur->tag_id, cur->offset, cur->length);
        
        // 简单的对齐检查打印
        if (cur->offset % 4 != 0) {
            printf(" [WARNING: Unaligned!]");
        }
        printf("\n");
        
        cur = cur->next;
    }
    printf("----------------------\n");
}