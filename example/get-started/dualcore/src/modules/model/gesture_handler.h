#ifndef __GESTURE_HANDLER_H__
#define __GESTURE_HANDLER_H__
/******gesture_handler.h*****/
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#define GESTURE_EVENT_PRESS (1 << 0)
#define GESTURE_EVENT_TAP (1 << 1)
#define GESTURE_EVENT_HOLD (1 << 2)
#define GESTURE_EVENT_FINGER_RELEASE (1 << 3)
#define GESTURE_EVENT_FORCE_RELEASE (1 << 4)
#define GESTURE_EVENT_LONGPRESS (1 << 5)
#define GESTURE_EVENT_BACK (1 << 6)
#define GESTURE_EVENT_WRIST_PRONATION (1 << 7)
#define GESTURE_EVENT_HAND_RELEASE (1 << 8)
#define GESTURE_EVENT_MOVE_FORWARD (1 << 9)
#define GESTURE_EVENT_MOVE_BACKWARD (1 << 10)

#define GESTURE_ALL_EVENTS (GESTURE_EVENT_PRESS | GESTURE_EVENT_TAP | GESTURE_EVENT_HOLD |                         \
                            GESTURE_EVENT_FINGER_RELEASE | GESTURE_EVENT_FORCE_RELEASE | GESTURE_EVENT_LONGPRESS | \
                            GESTURE_EVENT_BACK | GESTURE_EVENT_WRIST_PRONATION | GESTURE_EVENT_HAND_RELEASE |      \
                            GESTURE_EVENT_MOVE_FORWARD | GESTURE_EVENT_MOVE_BACKWARD)

    typedef enum
    {
        gesture_unknown,
        gesture_press,
        gesture_tap,
        gesture_hold,
        gesture_finger_release,
        gesture_longpress,
        gesture_back,
        gesture_double_tap,
        gesture_wrist_pronation,
        gesture_hand_release,
        // gesture_forward,
        // gesture_backward,
    } gesture_detect_state_t;

    extern void send_virtual_gesture_event(rt_uint32_t e);

#ifdef SOC_BF0_HCPU
#define ENABLE_VIRTUAL_TOUCH 0
#define USE_FFT_FILTER 0
#define GESTURE_DELAYED_JUDGMENT 1
#define GESTURE_JUDGMENT 0

    extern void force_release_finger(void);
    extern bool open_gesture_model(void);
    extern void set_tap_type(bool type);

#endif
#ifdef __cplusplus
}
#endif

#endif //__GESTURE_HANDLER_H__
