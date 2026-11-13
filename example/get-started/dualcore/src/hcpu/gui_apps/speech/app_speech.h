/*********************
 *      INCLUDES
 *********************/
#ifndef _APP_SPEECH_H
#define _APP_SPEECH_H

#include <stdint.h>
#include <stdbool.h>
bool app_speech_is_notification_reply(void);

void app_speech_data_init(void);
void app_speech_set_title(const uint8_t *s);
void app_speech_set_content(const uint8_t *s);

#endif /* _APP_SPEECH_H */
