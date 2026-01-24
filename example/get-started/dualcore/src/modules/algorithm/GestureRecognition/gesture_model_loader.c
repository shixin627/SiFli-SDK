/**
 ******************************************************************************
 * @file   gesture_model_loader.c
 * @author Skaiwalk software development team
 * @brief  Dynamic model loader implementation for gesture recognition
 ******************************************************************************
 */
/**
 * Copyright (c) 2024 - 2025, Skaiwalk Technology
 * All rights reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gesture_model_loader.h"
#include "gesture_detect_model_data.h"
#include "rtthread.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "bloc_filesystem.h"
#include <unistd.h>

#define DBG_TAG "MODEL.LOADER"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* Global model structures */
static gesture_model_t tap_model = {0};
static gesture_model_t release_model = {0};

/**
 * @brief Check if a file exists
 * @param path File path to check
 * @return true if file exists, false otherwise
 */
static bool file_exists(const char *path)
{
    struct stat st;
    return (stat(path, &st) == 0);
}

/**
 * @brief Load model from file system
 * @param file_path Path to the model file
 * @param model Pointer to model structure
 * @return 0 on success, negative on error
 */
static int load_model_from_file(const char *file_path, gesture_model_t *model)
{
    int fd = -1;
    struct stat file_stat;
    size_t bytes_read;

    if (!file_path || !model)
    {
        LOG_E("Invalid parameters");
        return -1;
    }

    /* Check if file exists */
    if (!file_exists(file_path))
    {
        LOG_W("Model file not found: %s", file_path);
        return -2;
    }

    /* Get file size */
    if (stat(file_path, &file_stat) != 0)
    {
        LOG_E("Failed to get file stats: %s", file_path);
        return -3;
    }

    if (file_stat.st_size > MAX_MODEL_SIZE)
    {
        LOG_E("Model file too large: %d bytes (max: %d)", file_stat.st_size, MAX_MODEL_SIZE);
        return -4;
    }

    /* Allocate memory for model */
    model->data = (uint8_t *)rt_malloc(file_stat.st_size);
    if (!model->data)
    {
        LOG_E("Failed to allocate memory for model (%d bytes)", file_stat.st_size);
        return -5;
    }

    /* Open and read the model file */
    fd = open(file_path, O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        LOG_E("Failed to open model file: %s", file_path);
        rt_free(model->data);
        model->data = NULL;
        return -6;
    }

    bytes_read = read(fd, model->data, file_stat.st_size);
    close(fd);

    if (bytes_read != file_stat.st_size)
    {
        LOG_E("Failed to read model file completely: read %d/%d bytes", bytes_read, file_stat.st_size);
        rt_free(model->data);
        model->data = NULL;
        return -7;
    }

    model->size = file_stat.st_size;
    model->is_dynamic = true;
    model->is_loaded = true;

    LOG_I("Successfully loaded model from %s (%d bytes)", file_path, model->size);
    return 0;
}

/**
 * @brief Initialize the model loader
 * @return 0 on success, negative on error
 */
int gesture_model_loader_init(void)
{
    LOG_I("Initializing gesture model loader");

    /* Initialize model structures */
    memset(&tap_model, 0, sizeof(gesture_model_t));
    memset(&release_model, 0, sizeof(gesture_model_t));

    return 0;
}

/**
 * @brief Load tap detection model from file system or use default
 * @param model Pointer to model structure to initialize
 * @return 0 on success, negative on error
 */
int load_tap_model(gesture_model_t *model)
{
    int ret;

    /* If model is already loaded, return success */
    if (model->is_loaded)
    {
        LOG_D("Tap model already loaded");
        return 0;
    }

    /* Try to load from file system first */
    ret = load_model_from_file(GESTURE_TAP_MODEL_PATH, model);
    if (ret == 0)
    {
        LOG_I("Using dynamic tap model from file system");
        return 0;
    }

    /* Fall back to default embedded model */
    LOG_I("Using default embedded tap model");
    model->data = (uint8_t *)g_gesture_detect_model_data;
    model->size = g_gesture_detect_model_data_len;
    model->is_dynamic = false;
    model->is_loaded = true;

    return 0;
}

/**
 * @brief Load release detection model from file system or use default
 * @param model Pointer to model structure to initialize
 * @return 0 on success, negative on error
 */
int load_release_model(gesture_model_t *model)
{
    int ret;

    if (!model)
    {
        model = &release_model;
    }

    /* If model is already loaded, return success */
    if (model->is_loaded)
    {
        LOG_D("Release model already loaded");
        return 0;
    }

    /* Try to load from file system first */
    ret = load_model_from_file(GESTURE_RELEASE_MODEL_PATH, model);
    if (ret == 0)
    {
        LOG_I("Using dynamic release model from file system");
        return 0;
    }

    /* Fall back to default embedded model */
    LOG_I("Using default embedded release model");
    model->data = (uint8_t *)g_gesture_detect_release_model_data;
    model->size = g_gesture_detect_release_model_data_len;
    model->is_dynamic = false;
    model->is_loaded = true;

    return 0;
}

/**
 * @brief Unload dynamically loaded model and free memory
 * @param model Pointer to model structure to unload
 * @return 0 on success, negative on error
 */
int unload_model(gesture_model_t *model)
{
    /* Only free dynamically allocated models */
    if (model->is_dynamic && model->data)
    {
        rt_free(model->data);
    }

    /* Clear model structure */
    model->data = NULL;
    model->size = 0;
    model->is_dynamic = false;
    model->is_loaded = false;

    return 0;
}

/**
 * @brief Get the tap model data pointer
 * @return Pointer to tap model data
 */
const unsigned char *get_tap_model_data(void)
{
    /* Load model if not already loaded */
    if (!tap_model.is_loaded)
    {
        load_tap_model(&tap_model);
    }

    return tap_model.data;
}

/**
 * @brief Get the tap model data size
 * @return Size of tap model data
 */
uint32_t get_tap_model_size(void)
{
    /* Load model if not already loaded */
    if (!tap_model.is_loaded)
    {
        load_tap_model(&tap_model);
    }

    return tap_model.size;
}

/**
 * @brief Get the release model data pointer
 * @return Pointer to release model data
 */
const unsigned char *get_release_model_data(void)
{
    /* Load model if not already loaded */
    if (!release_model.is_loaded)
    {
        load_release_model(&release_model);
    }

    return release_model.data;
}

/**
 * @brief Get the release model data size
 * @return Size of release model data
 */
uint32_t get_release_model_size(void)
{
    /* Load model if not already loaded */
    if (!release_model.is_loaded)
    {
        load_release_model(&release_model);
    }

    return release_model.size;
}

int unload_tap_model(void)
{
    return unload_model(&tap_model);
}

int unload_release_model(void)
{
    return unload_model(&release_model);
}
/**
 * @brief Clear (delete) the tap model file from file system
 * @return 0 on success, negative on error
 */
void clear_tap_model_file(void)
{
    bloc_file_system.delete_file(GESTURE_TAP_MODEL_PATH);
}

void clear_release_model_file(void)
{
    bloc_file_system.delete_file(GESTURE_RELEASE_MODEL_PATH);
}

/************************ (C) COPYRIGHT Skaiwalk Technology *******END OF FILE****/
