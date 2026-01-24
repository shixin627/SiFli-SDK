#ifndef __WRISTBAND_UPDATE_FONT_H__
#define __WRISTBAND_UPDATE_FONT_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "stdbool.h"

void WristBandFontBlockInit(void);
bool WristBandBinaryFontStore(uint8_t *buf, uint16_t len);
void watch_binary_font_update_handler(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
