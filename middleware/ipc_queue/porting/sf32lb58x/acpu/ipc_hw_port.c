/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include "ipc_hw.h"

static void lcpu2acpu_notification_callback(MAILBOX_HandleTypeDef *hmailbox, uint8_t q_idx);
static void hcpu2acpu_notification_callback(MAILBOX_HandleTypeDef *hmailbox, uint8_t q_idx);

ipc_hw_t ipc_hw_obj =
{
    .ch =
    {
        [0] =
        {
            .cfg =
            {
                .rx =
                {
                    .handle.Instance = L2A_MAILBOX,
                    .handle.NotificationCallback = lcpu2acpu_notification_callback,
                    .core  = CORE_ID_LCPU,
                    .irqn = LCPU2ACPU_IRQn,
                },
                .tx =
                {
                    .handle.Instance = A2L_MAILBOX,
                    .core  = CORE_ID_ACPU,
                    .irqn = ACPU2LCPU_IRQn,

                }
            }
        },
        [1] =
        {
            .cfg =
            {
                .rx =
                {
                    .handle.Instance = H2A_MAILBOX,
                    .handle.NotificationCallback = hcpu2acpu_notification_callback,
                    .core  = CORE_ID_ACPU,
                    .irqn = HCPU2ACPU_IRQn,
                },
                .tx =
                {
                    .handle.Instance = A2H_MAILBOX,
                    .core  = CORE_ID_HCPU,
                    .irqn = ACPU2HCPU_IRQn,
                }
            }
        },
    }
};

static void lcpu2acpu_notification_callback(MAILBOX_HandleTypeDef *hmailbox, uint8_t q_idx)
{
    SF_ASSERT(q_idx < IPC_HW_QUEUE_NUM);

    if (ipc_hw_obj.ch[0].data.act_bitmap & (1UL << q_idx))
    {
        ipc_queue_data_ind(ipc_hw_obj.ch[0].data.user_data[q_idx]);
    }
}

static void hcpu2acpu_notification_callback(MAILBOX_HandleTypeDef *hmailbox, uint8_t q_idx)
{
    SF_ASSERT(q_idx < IPC_HW_QUEUE_NUM);

    if (ipc_hw_obj.ch[1].data.act_bitmap & (1UL << q_idx))
    {
        ipc_queue_data_ind(ipc_hw_obj.ch[1].data.user_data[q_idx]);
    }
}


void LCPU2ACPU_IRQHandler(void)
{
    /* enter interrupt */
    os_interrupt_enter();
    HAL_MAILBOX_IRQHandler(&ipc_hw_obj.ch[0].cfg.rx.handle);
    /* leave interrupt */
    os_interrupt_exit();
}

void HCPU2ACPU_IRQHandler(void)
{
    /* enter interrupt */
    os_interrupt_enter();
    HAL_MAILBOX_IRQHandler(&ipc_hw_obj.ch[1].cfg.rx.handle);
    /* leave interrupt */
    os_interrupt_exit();
}


