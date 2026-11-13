/* SPDX-License-Identifier: Apache-2.0 */
#ifndef INSTRUCTION_LIST_RENDER_MEMORY_H
#define INSTRUCTION_LIST_RENDER_MEMORY_H
#include <stdbool.h>
#include <stddef.h>
typedef struct instruction_render_pool instruction_render_pool_t;
instruction_render_pool_t *instruction_render_acquire(void);
void instruction_render_release(instruction_render_pool_t *pool);
void *instruction_render_enter(const void *owner);
void *instruction_render_use(instruction_render_pool_t *pool);
void instruction_render_leave(void *previous);
bool instruction_render_failed(instruction_render_pool_t *pool);
size_t instruction_render_peak(instruction_render_pool_t *pool);
size_t instruction_render_reserved(void);
size_t instruction_render_high_water(void);
#endif