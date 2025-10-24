/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CORE_DUMP_H__
#define __CORE_DUMP_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Perfrom core dump operation, such as save context to flash for offline analysis
 *
  * @return None
 */
void core_dump(void);

/**
 * @brief Perfrom core dump extended operation which is implemented by user
 *
 * @return None
 */
void core_dump_ext(void);

#ifdef __cplusplus
}
#endif


#endif /* __CORE_DUMP_H__ */

