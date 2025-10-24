/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "board.h"
#include "core_dump.h"


void core_dump(void)
{
#ifdef USING_CORE_DUMP_EXT
    /* call user-defined core dump function */
    core_dump_ext();
#endif /* USING_CORE_DUMP_EXT */
}