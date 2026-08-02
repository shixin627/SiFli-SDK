#ifndef __BLOC_SETTING_H__
#define __BLOC_SETTING_H__
/* bloc_setting.h */
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "stdint.h"
#include "watch_global_data.h"

    typedef struct
    {
        void (*set_language)(const char *language);      // 函數用於設置語言
        void (*notify_language)(void);                   // 函數用於通知當前語言
        void (*set_dnd_status)(uint8_t status);          // 函數用於設置免打擾狀態
        void (*set_watch_face)(T_CLOCK_MENU_TYPE index); // 函數用於設置手錶面板
        void (*set_hour_format)(uint8_t format);         // 函數用於設置時間格式（12小時或24小時）
        void (*set_watch_time)(T_UTC_TIME time);         // 函數用於設置手錶時間
        void (*set_lift_switch_status)(bool status);     // 函數用於設置舉手亮屏開關狀態
        void (*set_wear_detect_off)(bool off);           // 函數用於設置佩戴偵測停用(診斷用,停用時除充電外都算配戴)
        void (*set_hr_continuous)(bool enable);          // 連續心率量測(診斷用,同運動 app 的量法:PPG 常開、演算法不重啟)
        void (*set_brightness)(uint8_t percent);         // 函數用於設置屏幕亮度
        void (*set_screen_time)(uint8_t time);           // 函數用於設置屏幕時間
        void (*set_user_profile)(userprofile_union_t profile);
        uint8_t (*get_power_save_mode)(void);
        void (*set_power_save_mode)(uint8_t mode);

    } SettingProvider;
    extern SettingProvider setting_provider;
    extern void bloc_setting_load_watch_system(void);
    extern char *bloc_setting_get_language(void);
    extern void bloc_setting_set_gesture_detect_threshold(int threshold);
    extern int bloc_setting_get_gesture_detect_threshold(void);
#ifdef __cplusplus
}
#endif

#endif //__BLOC_SETTING_H__
