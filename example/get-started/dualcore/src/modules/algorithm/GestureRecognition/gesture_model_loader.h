/**
 ******************************************************************************
 * @file   gesture_model_loader.h
 * @author Skaiwalk software development team
 * @brief  Dynamic model loader for gesture recognition
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 */

#ifndef GESTURE_MODEL_LOADER_H
#define GESTURE_MODEL_LOADER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

/* Model file paths in flash file system */
#define GESTURE_TAP_MODEL_PATH "/model/gesture_tap.tflite"
#define GESTURE_RELEASE_MODEL_PATH "/model/gesture_release.tflite"

/* Maximum model size (adjust based on your models) */
#define MAX_MODEL_SIZE (10 * 1024) // 10KB

    /**
     * @brief Model data structure
     */
    typedef struct
    {
        uint8_t *data;      // Pointer to model data
        uint32_t size;      // Size of model data
        bool is_dynamic;    // Whether the model was dynamically loaded
        bool is_loaded;     // Whether the model is currently loaded
    } gesture_model_t;

    /**
     * @brief Initialize the model loader
     * @return 0 on success, negative on error
     */
    int gesture_model_loader_init(void);

    /**
     * @brief Load tap detection model from file system or use default
     * @param model Pointer to model structure to initialize
     * @return 0 on success, negative on error
     */
    int load_tap_model(gesture_model_t *model);

    /**
     * @brief Load release detection model from file system or use default
     * @param model Pointer to model structure to initialize
     * @return 0 on success, negative on error
     */
    int load_release_model(gesture_model_t *model);

    /**
     * @brief Unload dynamically loaded model and free memory
     * @param model Pointer to model structure to unload
     * @return 0 on success, negative on error
     */
    int unload_model(gesture_model_t *model);

    /**
     * @brief Get the tap model data pointer
     * @return Pointer to tap model data
     */
    const unsigned char *get_tap_model_data(void);

    /**
     * @brief Get the tap model data size
     * @return Size of tap model data
     */
    uint32_t get_tap_model_size(void);

    /**
     * @brief Get the release model data pointer
     * @return Pointer to release model data
     */
    const unsigned char *get_release_model_data(void);

    /**
     * @brief Get the release model data size
     * @return Size of release model data
     */
    uint32_t get_release_model_size(void);

    int unload_tap_model(void);
    int unload_release_model(void);

    /**
     * @brief Clear (delete) the tap model file from file system
     * @return 0 on success, negative on error
     */
    void clear_tap_model_file(void);

    /**
     * @brief Clear (delete) the release model file from file system
     * @return 0 on success, negative on error
     */
    void clear_release_model_file(void);

#ifdef __cplusplus
}
#endif

#endif // GESTURE_MODEL_LOADER_H
