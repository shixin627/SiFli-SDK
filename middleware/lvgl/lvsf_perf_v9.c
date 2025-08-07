#ifdef PKG_USING_SEGGER_RTT
#include "littlevgl2rtt.h"
#include "Global_type.h"
#include "SEGGER_SYSVIEW.h"

static bool systemview_lv_debug_en = true;
extern void lv_enter_func(U32 id, const char *task_name, U32 param1, U32 param2);
extern void lv_exit_func(U32 id);
void lv_debug_task_start_exec_v9(const void *func, const char *func_name)
{
    char task_name[64];
    if (!systemview_lv_debug_en) return;

    rt_sprintf(task_name, "%s %p", func_name ? func_name : "func", func);

    lv_enter_func((U32)func, &task_name[0], 0, 0);
}

void lv_debug_task_stop_exec_v9(const void *func)
{
    if (!systemview_lv_debug_en) return;

    lv_exit_func((U32)func);
}



void lv_debug_mark_start_v9(uint32_t id, const char *desc)
{
    if (!systemview_lv_debug_en) return;

    SEGGER_SYSVIEW_OnUserStart(id);
    if (desc) SEGGER_SYSVIEW_Print(desc);

}


void lv_debug_mark_stop_v9(uint32_t id)
{
    if (!systemview_lv_debug_en) return;

    //SEGGER_SYSVIEW_OnTaskStopReady((U32) id, 1);

    //SEGGER_SYSVIEW_OnTaskStartExec((U32) id);
    //SEGGER_SYSVIEW_OnTaskStopExec();


    SEGGER_SYSVIEW_OnUserStop(id);
}
#endif /* PKG_USING_SEGGER_RTT */