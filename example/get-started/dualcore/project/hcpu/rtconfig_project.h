#ifndef RTCONFIG_PROJECT_H__
#define RTCONFIG_PROJECT_H__

#if defined(PKG_USING_QUICKJS) && !defined(_MSC_VER)
    /* Re-enable QuickJS's stack-overflow guard, which the vendored source
     * disables for RT-Thread builds:
     *
     *   #if !defined(EMSCRIPTEN) && !defined(BSP_USING_RTTHREAD)
     *   #define CONFIG_STACK_CHECK
     *   #endif                             -- external/quickjs/quickjs.c:81
     *
     * Without it JS_SetMaxStackSize() stores a number nobody reads and
     * js_check_stack_overflow() is compiled down to "return FALSE", so
     * `function f(){return f();} f();` in a third-party app runs off the native
     * stack. On the simulator that kills the process; on the watch it is a hard
     * fault. Either way it defeats the ADR-0019 sandbox, whose whole promise is
     * that a bad app cannot take the watch down.
     *
     * Defined here rather than by patching external/: rtconfig.h is force-
     * included ahead of quickjs.c's own guard, so the macro is already set when
     * that #if runs and the vendor tree stays untouched.
     *
     * armclang only: with CONFIG_STACK_CHECK on, MSVC's js_get_stack_pointer()
     * returns _ReturnAddress() -- a code address, not a stack address
     * (external/quickjs/quickjs.c:1603). js_check_stack_overflow() then
     * compares it against rt->stack_limit, which is derived from a real stack
     * address, so the guard fires at arbitrary points including inside GC and
     * allocation. The armclang path uses __builtin_frame_address(0) and is
     * correct. */
    #define CONFIG_STACK_CHECK
#endif

/* Turn on just enough mbedtls for install-time package verification
 * (ADR-0019 §2.5): SHA-256 is already on in external/mbedtls/include/mbedtls/
 * config.h, but ECDSA and the P-256 curve are not — the curve is defined only
 * under PKG_USING_SM, and MBEDTLS_ECDSA_C appears solely inside a doc comment.
 *
 * Defined here for the same reason as CONFIG_STACK_CHECK above: rtconfig.h is
 * force-included ahead of config.h, config.h never defines these in its active
 * block, so nothing is redefined and the vendor tree stays untouched. The
 * matching source files are compiled by src/modules/sdk/SConscript rather than
 * by external/mbedtls/SConscript, which builds a much wider set. */
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ASN1_WRITE_C

/* Override the SDK default "SifliDemo" Classic-BT local-name prefix. */
#define BT_LOCAL_NAME_PREFIX "Skaiwalk Air"
/* Use the prefix verbatim as the Classic-BT friendly name (no "-<mac>" suffix),
   to match the unified BLE device name. Consumed in bts2_app_generic.c. */
#define BT_LOCAL_NAME_NO_SUFFIX

#if defined(_MSC_VER)
    /* APP_TRANS_ANIMATION_OVERWRITE is now set via Kconfig (proj.conf), no
     * need to define it here. */

    /* watch_sys_service.h gates HCPU-side struct members on SOC_BF0_HCPU
     * (vs the LCPU set). PC sim is single-core acting as HCPU. */
    #define SOC_BF0_HCPU

    /* BSP_USING_COMMUNICATE is gated by BSP_BLE_SIBLES in Kconfig — but PC
     * sim wants to compile the comm/parse state machine so app code that
     * #ifdef's on it still works. Force-enable here. */
    #define BSP_USING_COMMUNICATE
    #define BSP_USING_COMM_BOND
    #define BSP_USING_COMM_CONTROL
    #define BSP_USING_COMM_HEALTH
    #define BSP_USING_COMM_NOTIFY
    #define BSP_USING_COMM_SETTING
    #define BSP_USING_COMM_SKAILINK

    #define RT_HEAP_SIZE   (680000)
    #define NORESOURCE  /* RT_VERSION in winuser.h */
    #define _CRT_ERRNO_DEFINED  /* errno macro redefinition */
    /* Note: don't define _INC_TIME_INL / _INC_WTIME_INL — they're MSVC's
     * own include-once guards. Pre-defining them suppresses the time.h
     * declarations of time(), localtime(), etc. */

    /* disable some warnings in MSC */
    #pragma warning(disable:4273)   /* warning C4273: inconsistent dll linkage */
    #pragma warning(disable:4312)
    #pragma warning(disable:4311)
    #pragma warning(disable:4996)   /* POSIX name deprecated */
    #pragma warning(disable:4267)
    #pragma warning(disable:4244)
#endif /* _MSC_VER */

/* 瘦身(founder 2026-10-06:不動功能、拿掉沒用到的):這些 LVGL 元件沒有任何 app 用到,只被 lvsf_theme_1.c 的
   樣式設定順手引用。proj.conf 已設 "# CONFIG_LV_USE_X is not set",但這個 SDK 的 lv_conf_internal.h 在 Kconfig
   未啟用時(沒有 CONFIG_LV_COLOR_DEPTH)會把沒定義的元件預設成 1,所以要在這裡明確定成 0 才真的不編。
   省 ~26.5 KB(main.bin 2,618,400 -> 2,591,312)。 */
#define LV_USE_CHART 0
#define LV_USE_COLORWHEEL 0
#define LV_USE_TABLE 0
#define LV_USE_METER 0
#define LV_USE_KEYBOARD 0
#define LV_USE_MENU 0
#define LV_USE_SPINBOX 0
#define LV_USE_TABVIEW 0
#define LV_USE_LED 0
#define LV_USE_WIN 0
#define LV_USE_CALENDAR 0
#define LV_USE_IMGBTN 0
#define LV_USE_SPAN 0
#define LV_USE_ANIMIMG 0

#endif
