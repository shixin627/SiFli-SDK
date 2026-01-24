/**
 ******************************************************************************
 * @file   gesture_model_config.h
 * @author Skaiwalk software development team
 * @brief  Configuration for gesture recognition models
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 */

#ifndef GESTURE_MODEL_CONFIG_H
#define GESTURE_MODEL_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Model file paths in flash file system */
#define GESTURE_MODEL_DIR "/model"
#define GESTURE_TAP_MODEL_FILE "gesture_tap.tflite"
#define GESTURE_RELEASE_MODEL_FILE "gesture_release.tflite"

/* Full paths */
#define GESTURE_TAP_MODEL_FULLPATH GESTURE_MODEL_DIR "/" GESTURE_TAP_MODEL_FILE
#define GESTURE_RELEASE_MODEL_FULLPATH GESTURE_MODEL_DIR "/" GESTURE_RELEASE_MODEL_FILE

/* Model size limits */
#define MAX_TAP_MODEL_SIZE (10 * 1024)      // 10KB
#define MAX_RELEASE_MODEL_SIZE (10 * 1024)  // 10KB

/**
 * @brief Model update status
 */
typedef enum
{
    MODEL_UPDATE_SUCCESS = 0,
    MODEL_UPDATE_FILE_NOT_FOUND = -1,
    MODEL_UPDATE_FILE_TOO_LARGE = -2,
    MODEL_UPDATE_READ_ERROR = -3,
    MODEL_UPDATE_INVALID_MODEL = -4,
    MODEL_UPDATE_MEMORY_ERROR = -5
} model_update_status_t;

#ifdef __cplusplus
}
#endif

#endif // GESTURE_MODEL_CONFIG_H
