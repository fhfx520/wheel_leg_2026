#include "container.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define INITIAL_CAPACITY 512
#define ALIGN_SIZE 4
#define ALIGN_UP(size) (((size) + (ALIGN_SIZE - 1)) & ~(ALIGN_SIZE - 1))

// ---------------------------------------------------------
// 内部结构定义
// ---------------------------------------------------------

typedef struct Node {
    uint32_t tag_id;
    ContainerDataType type;
    size_t offset;
    size_t length;
    uint32_t frame_id;   
    uint32_t read_count; 
    struct Node* next;
} Node;

struct Container {
    Node* head;
    uint8_t* buffer;
    size_t capacity;
    size_t used_size;
    
    // --- 新增互斥锁 ---
    SemaphoreHandle_t mutex;
};

// ---------------------------------------------------------
// 内部辅助 (调用前必须已上锁)
// ---------------------------------------------------------

static Node* find_node(Container* ctx, uint32_t tag_id) {
    Node* cur = ctx->head;
    while (cur) {
        if (cur->tag_id == tag_id) return cur;
        cur = cur->next;
    }
    return NULL;
}

static int ensure_capacity(Container* ctx, size_t required_len) {
    size_t needed_total = ctx->used_size + required_len;

    if (ctx->buffer == NULL) {
        size_t start_size = (required_len > INITIAL_CAPACITY) ? required_len : INITIAL_CAPACITY;
        start_size = ALIGN_UP(start_size);
        ctx->buffer = (uint8_t*)malloc(start_size);
        if (!ctx->buffer) return -1;
        ctx->capacity = start_size;
        return 0;
    }

    if (needed_total > ctx->capacity) {
        size_t new_cap = ctx->capacity;
        while (new_cap < needed_total) new_cap *= 2; 
        
        uint8_t* new_buf = (uint8_t*)realloc(ctx->buffer, new_cap);
        if (!new_buf) return -1;
        
        ctx->buffer = new_buf;
        ctx->capacity = new_cap;
    }
    return 0;
}

// ---------------------------------------------------------
// API 实现
// ---------------------------------------------------------

Container* container_create(void) {
    Container* ctx = (Container*)malloc(sizeof(Container));
    if (!ctx) return NULL;
    memset(ctx, 0, sizeof(Container));
    
    // 创建 FreeRTOS 互斥量
    ctx->mutex = xSemaphoreCreateMutex();
    if (ctx->mutex == NULL) {
        free(ctx);
        return NULL;
    }

    return ctx;
}

void container_destroy(Container* ctx) {
    if (!ctx) return;

    // 删除锁
    if (ctx->mutex) {
        vSemaphoreDelete(ctx->mutex);
    }

    Node* cur = ctx->head;
    while (cur) {
        Node* tmp = cur;
        cur = cur->next;
        free(tmp);
    }
    if (ctx->buffer) free(ctx->buffer);
    free(ctx);
}

int container_set(Container* ctx, uint32_t tag_id, const void* data, size_t data_len, ContainerDataType type) {
    if (!ctx || !data || data_len == 0) return -1;

    // === 上锁 ===
    if (xSemaphoreTake(ctx->mutex, portMAX_DELAY) != pdTRUE) {
        return -1; // 获取锁失败
    }

    size_t current_offset = ctx->used_size;
    size_t aligned_offset = ALIGN_UP(current_offset);
    size_t padding = aligned_offset - current_offset;
    size_t total_needed = padding + data_len;

    if (ensure_capacity(ctx, total_needed) != 0) {
        xSemaphoreGive(ctx->mutex); // 失败也要释放锁
        return -1; 
    }

    Node* node = find_node(ctx, tag_id);

    memcpy(ctx->buffer + aligned_offset, data, data_len);
    ctx->used_size = aligned_offset + data_len;

    if (node) {
        node->offset = aligned_offset;
        node->length = data_len;
        node->type = type;
        node->frame_id++; 
    } else {
        Node* new_node = (Node*)malloc(sizeof(Node));
        // 这里简化了错误处理，实际应检查 malloc 失败
        new_node->tag_id = tag_id;
        new_node->type = type;
        new_node->offset = aligned_offset;
        new_node->length = data_len;
        new_node->frame_id = 1;     
        new_node->read_count = 0;
        new_node->next = ctx->head;
        ctx->head = new_node;
    }

    // === 解锁 ===
    xSemaphoreGive(ctx->mutex);
    return 0;
}

int container_get(Container* ctx, uint32_t tag_id, void** out_data, size_t* out_len, ContainerDataType* out_type) {
    if (!ctx || !out_data) return -1;

    // === 上锁 ===
    if (xSemaphoreTake(ctx->mutex, portMAX_DELAY) != pdTRUE) return -1;

    Node* node = find_node(ctx, tag_id);
    if (!node) {
        xSemaphoreGive(ctx->mutex);
        return -1; 
    }

    *out_data = (ctx->buffer + node->offset);
    if (out_len) *out_len = node->length;
    if (out_type) *out_type = node->type;
    node->read_count++; 

    // === 解锁 ===
    xSemaphoreGive(ctx->mutex);
    return 0;
}

int container_get_stat(Container* ctx, uint32_t tag_id, ContainerStat* out_stat) {
    if (!ctx || !out_stat) return -1;

    // === 上锁 ===
    if (xSemaphoreTake(ctx->mutex, portMAX_DELAY) != pdTRUE) return -1;

    Node* node = find_node(ctx, tag_id);
    if (!node) {
        xSemaphoreGive(ctx->mutex);
        return -1; 
    }

    out_stat->frame_id = node->frame_id;
    out_stat->read_count = node->read_count;
    out_stat->length = node->length;

    // === 解锁 ===
    xSemaphoreGive(ctx->mutex);
    return 0;
}

void container_dump_info(Container* ctx) {
    if (!ctx) return;
    // 调试打印也建议上锁，防止打印一半链表变了
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    
    printf("\n--- Container Dump ---\n");
    printf("Capacity : %zu bytes\n", ctx->capacity);
    Node* cur = ctx->head;
    while(cur) {
        printf("  [ID:%u] FrmID:%u Off:%zu\n", cur->tag_id, cur->frame_id, cur->offset);
        cur = cur->next;
    }
    printf("----------------------\n");
    
    xSemaphoreGive(ctx->mutex);
}