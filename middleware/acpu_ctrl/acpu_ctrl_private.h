/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ACPU_CTRL_PRIVATE_H__
#define __ACPU_CTRL_PRIVATE_H__


#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t is_rsp: 1; /**< response task flag */
    uint32_t task_id: 31;
    uint32_t task_param_size;
    void *task_param;

    void *sema;   /**< task caller blocking semaphore */

    uint32_t ret_error_code;
    uint32_t ret_value;
} acpu_ctrl_ipc_msg_t;


#ifdef __cplusplus
}
#endif
#endif /* __ACPU_CTRL_PRIVATE_H__ */