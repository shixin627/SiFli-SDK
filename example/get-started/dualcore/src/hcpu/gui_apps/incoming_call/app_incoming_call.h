#ifndef __APP_INCOMING_CALL_H__
#define __APP_INCOMING_CALL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

void incoming_call_set_caller(const char *title, const char *id, uint8_t type);
void incoming_call_close_if_active(const char *id);

#ifdef __cplusplus
}
#endif

#endif
