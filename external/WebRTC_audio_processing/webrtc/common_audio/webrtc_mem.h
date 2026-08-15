/*
 * SPDX-FileCopyrightText: 2022-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WEBRTC_MEM_H
#define __WEBRTC_MEM_H

#include <rtthread.h>

#ifdef AUDIO
    /* Skaiwalk: route ALL WebRTC allocations to a dedicated PSRAM memheap
       (bloc_v2t.c) instead of audio_mem_malloc. audio_mem_malloc is a bare
       rt_malloc + RT_ASSERT on the SRAM system heap, which runs ~99 % full
       during voice — and this port's AUDIO_MEM_ALLOC mode makes NS allocate
       ~a dozen scratch buffers per 10 ms frame, so the mic pipeline rebooted
       the watch with "audio_mem_malloc:163 (ptr)" under memory pressure.
       The PSRAM backend falls back to rt_malloc if the pool is unavailable,
       and returns NULL (never asserts) on true exhaustion. */
    extern void *webrtc_heap_malloc(unsigned int size);
    extern void webrtc_heap_free(void *ptr);
    extern void *webrtc_heap_calloc(unsigned int count, unsigned int size);
    extern void *webrtc_heap_realloc(void *ptr, unsigned int newsize);
    #undef malloc
    #undef free
    #undef calloc
    #undef realloc
    #define malloc(size)    webrtc_heap_malloc(size)
    #define free(ptr)       webrtc_heap_free(ptr)
    #define calloc(c,s)     webrtc_heap_calloc(c,s)
    #define realloc(m, n)   webrtc_heap_realloc(m, n)
#endif

#endif // __WEBRTC_MEM_H

