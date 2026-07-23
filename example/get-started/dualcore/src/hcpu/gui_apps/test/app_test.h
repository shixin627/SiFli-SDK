#ifndef __APP_TEST_H__
#define __APP_TEST_H__

#include <stdbool.h>

typedef struct
{
    void (*progress)(void *context, int percent, const char *stage,
                     const char *detail);
    void (*item)(void *context, const char *name, bool passed,
                 const char *detail);
    void (*done)(void *context, int failures);
    void *context;
} app_test_screening_callbacks_t;

/* UART-only hidden mode of the existing test app. */
bool app_test_board_screening_start(
    const app_test_screening_callbacks_t *callbacks);
bool app_test_board_screening_cancel(void);

#endif /* __APP_TEST_H__ */
