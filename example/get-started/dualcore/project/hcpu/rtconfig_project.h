#ifndef RTCONFIG_PROJECT_H__
#define RTCONFIG_PROJECT_H__

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

#endif
