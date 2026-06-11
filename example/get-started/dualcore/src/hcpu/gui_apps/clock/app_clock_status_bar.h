#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#include "time.h"
#include "app_clock_main.h"

extern void app_clock_main_status_bar_init(lv_obj_t *par);
extern void app_clock_main_status_bar_deinit(void);
extern void app_clock_ai_status_bar_init(lv_obj_t *par);
extern void app_clock_device_change_bar_init(lv_obj_t *par);
extern void app_clock_device_change_bar_open(void);
extern void app_clock_status_bar_return_home(void);
extern int app_clock_status_bar_pull_home(int16_t dx);
extern void app_clock_status_bar_pull_home_release(bool commit);
