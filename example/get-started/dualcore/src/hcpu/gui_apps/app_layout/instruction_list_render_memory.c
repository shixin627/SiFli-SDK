/* SPDX-License-Identifier: Apache-2.0 */
#include "instruction_list_render_memory.h"
#include <stdint.h>
#include <rtthread.h>
#include "lvgl.h"

/* One committed frame and one candidate. Never borrow SRAM on the watch. */
#define RENDER_BYTES (256u * 1024u)
#ifdef BSP_USING_PC_SIMULATOR
/* The simulator has a small system heap, but no PSRAM heap. Keep the same
   two bounded frame regions independent of its system allocations. */
static union { uint64_t align; uint8_t bytes[RENDER_BYTES]; } s_host_storage[2];
#endif
struct instruction_render_pool {
    struct rt_memheap heap;
    void *storage;
    bool failed;
};
static instruction_render_pool_t s_pools[2];
static instruction_render_pool_t *s_active;
static rt_thread_t s_owner;

static instruction_render_pool_t *pool_for(const void *pointer)
{
    unsigned i;
    for(i = 0; i < 2; i++)
        if(s_pools[i].storage && (uintptr_t)pointer >= (uintptr_t)s_pools[i].storage &&
           (uintptr_t)pointer < (uintptr_t)s_pools[i].storage + RENDER_BYTES) return &s_pools[i];
    return NULL;
}
static bool render_fallible(void) { return s_active && s_owner == rt_thread_self(); }
static void *render_alloc(size_t bytes)
{
    void *result;
    if(!render_fallible()) return rt_malloc(bytes);
    result = rt_memheap_alloc(&s_active->heap, bytes);
    if(!result) s_active->failed = true;
    return result;
}
static void render_free(void *pointer)
{
    if(pool_for(pointer)) rt_memheap_free(pointer);
    else rt_free(pointer);
}
static void *render_realloc(void *pointer, size_t bytes)
{
    instruction_render_pool_t *pool = pool_for(pointer);
    void *result;
    if(!pointer) return render_alloc(bytes);
    if(!pool) return rt_realloc(pointer, bytes); /* Shared parent metadata stays shared. */
    result = rt_memheap_realloc(&pool->heap, pointer, bytes);
    if(!result) pool->failed = true;
    return result;
}

instruction_render_pool_t *instruction_render_acquire(void)
{
    static bool installed;
    static const lv_mem_allocator_t allocator = {render_alloc, render_free, render_realloc, render_fallible};
    unsigned i;
    if(!installed) { lv_mem_set_allocator(&allocator); installed = true; }
    for(i = 0; i < 2; i++)
        if(!s_pools[i].storage)
        {
#ifdef BSP_USING_PC_SIMULATOR
            s_pools[i].storage = s_host_storage[i].bytes;
#else
            extern struct rt_memheap app_image_psram_memheap;
            s_pools[i].storage = rt_memheap_alloc(&app_image_psram_memheap, RENDER_BYTES);
#endif
            if(!s_pools[i].storage) return NULL;
            rt_memheap_init(&s_pools[i].heap, i ? "listv1" : "listv0", s_pools[i].storage, RENDER_BYTES);
            s_pools[i].failed = false;
            return &s_pools[i];
        }
    return NULL;
}
void instruction_render_release(instruction_render_pool_t *pool)
{
    if(!pool || !pool->storage) return;
    rt_memheap_detach(&pool->heap);
#ifndef BSP_USING_PC_SIMULATOR
    rt_memheap_free(pool->storage);
#endif
    pool->storage = NULL;
}
void *instruction_render_use(instruction_render_pool_t *pool)
{
    instruction_render_pool_t *previous = s_active;
    s_owner = rt_thread_self();
    s_active = pool;
    return previous;
}
void *instruction_render_enter(const void *owner) { return instruction_render_use(pool_for(owner)); }
void instruction_render_leave(void *previous) { s_active = previous; }
bool instruction_render_failed(instruction_render_pool_t *pool) { return !pool || pool->failed; }
size_t instruction_render_peak(instruction_render_pool_t *pool) { return pool ? pool->heap.max_used_size : 0; }
size_t instruction_render_reserved(void)
{
    return (s_pools[0].storage ? RENDER_BYTES : 0) + (s_pools[1].storage ? RENDER_BYTES : 0);
}
size_t instruction_render_high_water(void)
{
    size_t peak = 0;
    unsigned i;
    for (i = 0; i < 2; i++)
        if (s_pools[i].storage) peak += s_pools[i].heap.max_used_size;
    return peak;
}
