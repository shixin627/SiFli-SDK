#ifndef __WRISTBAND_PEDO_H__
#define __WRISTBAND_PEDO_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "stdbool.h"



extern void send_sport_data(void);
extern void WristBandPedoDataBlockInit(void);
extern bool WristBandPedoDataStore(uint8_t *buf, uint16_t len);
extern bool WristBandPedoDataRestore(uint8_t *buf, uint16_t len);
extern void minute_sport_handler(void);
extern void doNewTimeSettingForPedoData(uint32_t old_sec, uint32_t new_sec);



#ifdef __cplusplus
}
#endif

#endif
