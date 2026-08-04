/*
 * SPDX-FileCopyrightText: 2022-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __WEBRTC_MEM_H
#define __WEBRTC_MEM_H

#include <rtthread.h>

#ifdef AUDIO
    #include "audio_mem.h"
    #undef malloc
    #undef free
    #undef calloc
    #undef realloc
    #define malloc(size)    audio_mem_malloc(size)
    #define free(ptr)       audio_mem_free(ptr)
    #define calloc(c,s)     audio_mem_calloc(c,s)
    #define realloc(m, n)   audio_mem_realloc(m, n)
#endif

#endif // __WEBRTC_MEM_H

