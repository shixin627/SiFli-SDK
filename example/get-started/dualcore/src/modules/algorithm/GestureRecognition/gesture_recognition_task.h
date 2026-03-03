#ifndef GESTURE_RECOGNITION_TASK_H
#define GESTURE_RECOGNITION_TASK_H
#define TAP_TARGET_SAMPLE_NUM 35
#define RELEASE_TARGET_SAMPLE_NUM 35
#define DEFAULT_GESTURE_THRESHOLD 70
extern void force_release_finger(void);
extern int tap_recognition_score;
extern int release_recognition_score;
extern int gesture_recognition_threshold;
extern void set_gesture_recognition_threshold(int threshold);
extern int get_gesture_recognition_threshold(void);
#endif // GESTURE_RECOGNITION_TASK_H
