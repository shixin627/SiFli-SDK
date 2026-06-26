#ifndef LVSF_GESTURE_H
#define LVSF_GESTURE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lvsf_gesture_init(lv_obj_t *parent);
void lvsf_gesture_deinit(void);

void lvsf_gesture_set_image(uint32_t idx, const void *src_img);
void display_gesture_detect_objs(uint32_t idx, bool display);
void lvsf_gesture_disable(void);
void lvsf_gesture_enable(void);
void lvsf_gesture_bars_realign(void);
/* Raise the edge-back detector bars + the drag hint above a later full-screen lv_layer_top
   overlay, so the overlay's left-edge swipe reaches the detector AND the hint shows. */
void lvsf_gesture_bring_to_front(void);


/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_GESTURE_H*/






