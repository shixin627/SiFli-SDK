#ifndef __WRISTBAND_UPDATE_ICON_H__
#define __WRISTBAND_UPDATE_ICON_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "stdbool.h"

void WristBandIconBlockInit(void);
bool WristBandBinaryIconStore(uint8_t *buf, uint16_t len);
void watch_binary_icon_update_handler(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
