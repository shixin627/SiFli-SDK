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
#include <stddef.h>

/* Model file paths in flash file system */
#define GESTURE_RELEASE_MODEL_PATH "/model/gesture_release.tflite"

/* Manifest tracks installed-model versions (phone is the pusher; watch
   validates at boot). See model_manifest_* APIs below. */
#define MODEL_MANIFEST_PATH     "/model/manifest.json"
#define MODEL_MANIFEST_TMP_PATH "/model/manifest.json.tmp"

/* Logical model names used in the manifest and over-the-air protocol.
   Distinct from the file paths so the protocol stays stable if the on-disk
   path layout changes. */
#define MODEL_NAME_RELEASE "gesture_release"

/* Compiled-in baseline reported when no installed model is present. */
#define GESTURE_RELEASE_BUILTIN_VERSION "1.0.0"

#define MODEL_NAME_MAX_LEN    32
#define MODEL_VERSION_MAX_LEN 16
#define MODEL_MANIFEST_JSON_MAX_LEN 512

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
     * @brief Get the release model data pointer
     * @return Pointer to release model data
     */
    const unsigned char *get_release_model_data(void);

    /**
     * @brief Get the release model data size
     * @return Size of release model data
     */
    uint32_t get_release_model_size(void);

    int unload_release_model(void);

    /**
     * @brief Clear (delete) the release model file from file system
     * @return 0 on success, negative on error
     */
    void clear_release_model_file(void);

    /**
     * @brief Update one model entry in the manifest after a successful push.
     *        Atomic via tmp-file + rename. Phone calls KEY_MODEL_VERSION_SET
     *        right after the file's END_SYNC succeeds.
     * @param name    Logical model name (must match a known entry; e.g. "gesture_release")
     * @param version Version string the phone pushed (e.g. "1.2.3")
     * @param size    Expected file size in bytes — used for boot validation
     * @return 0 on success, negative on error
     */
    int model_manifest_set_version(const char *name, const char *version,
                                   uint32_t size);

    /**
     * @brief Build the manifest JSON to send back over BLE.
     *        Always lists every known model — entries missing from disk fall
     *        back to the compiled-in baseline version (so the phone always
     *        has something to compare against the cloud catalog).
     * @return Number of bytes written (excluding NUL), or negative on error.
     */
    int model_manifest_to_json(char *out_buf, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif // GESTURE_MODEL_LOADER_H
