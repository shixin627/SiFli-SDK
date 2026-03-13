/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#include <TensorFlowLite.h>
#include <cmath>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "gesture_detect_model_data.h"
#include "gesture_recognition_task.h"
#include "main_functions.h"
#include "constants.h"
#include "accelerometer_handler.h"
#include "gesture_predictor.h"
#include "gesture_model_loader.h"

const int numSamples = TAP_TARGET_SAMPLE_NUM;
#if kChannelReleaseNumber == 4
const int release_numSamples = RELEASE_TARGET_SAMPLE_NUM;
#else
const int release_numSamples = RELEASE_TARGET_SAMPLE_NUM;
#endif
static int samplesRead = 0;

namespace
{
    constexpr int kTensorArenaSize = 8 * 1024;
    constexpr int release_kTensorArenaSize = 8 * 1024;
    alignas(16) uint8_t tensor_arena[kTensorArenaSize];
    alignas(16) uint8_t release_tensor_arena[release_kTensorArenaSize];
    tflite::ErrorReporter *error_reporter = nullptr;
    tflite::ErrorReporter *release_error_reporter = nullptr;
    const tflite::Model *model = nullptr;
    const tflite::Model *release_model = nullptr;
    tflite::MicroInterpreter *interpreter = nullptr;
    tflite::MicroInterpreter *release_interpreter = nullptr;
}

void init_gesture_recognition_model(void)
{
    // static tflite::MicroErrorReporter micro_error_reporter;
    // error_reporter = &micro_error_reporter;

    // Initialize model loader and load tap model
    gesture_model_loader_init();
    const unsigned char *model_data = get_tap_model_data();

    model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        MicroPrintf(
            "Model provided is schema version %d not equal "
            "to supported version %d.",
            model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }
    // CNN
    static tflite::MicroMutableOpResolver<6> micro_op_resolver;
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddSoftmax();
    static tflite::MicroInterpreter static_interpreter(model, micro_op_resolver, tensor_arena, kTensorArenaSize);

    interpreter = &static_interpreter;

// 如果启用了硬件加速
#if defined(BSP_USING_NN_ACC)
// 创建并添加硬件加速委托
#endif

    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to Allocate Tensors");
        return;
    }
    MicroPrintf("Allocate Tensors Successfully\n");
    TfLiteTensor *model_input = interpreter->input(0);
    int dimension_size = model_input->dims->size;
    MicroPrintf("dims->size = %d\n", dimension_size);
    MicroPrintf("model_input->dims->data[0] = %d\n", model_input->dims->data[0]);
    MicroPrintf("model_input->dims->data[1] = %d\n", model_input->dims->data[1]);
    MicroPrintf("model_input->dims->data[2] = kChannelNumber = %d\n", model_input->dims->data[2]);
    model_input = interpreter->input(0);
    if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
        (model_input->dims->data[1] != numSamples) ||
        (model_input->dims->data[2] != kChannelNumber) ||
        (model_input->type != kTfLiteFloat32))
    {
        TF_LITE_REPORT_ERROR(error_reporter, "Bad input tensor parameters in model");
        return;
    }

    MicroPrintf("model input type = %d\n", model_input->type);

    int input_length = model_input->bytes / sizeof(float);
    MicroPrintf("model input length = %d\n", input_length);
    if (input_length != numSamples * kChannelNumber)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "error input length");
        return;
    }

    int output_length = interpreter->output(0)->bytes / sizeof(float);
    MicroPrintf("model output length = %d\n", output_length);
    if (output_length != kGestureCount)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "Bad output tensor parameters in model");
        return;
    }

    // extern void gesture_detect_test(tflite::MicroInterpreter * interpreter);
    // gesture_detect_test(interpreter);
}

void init_gesture_recognition_release_model(void)
{
    // static tflite::MicroErrorReporter micro_error_reporter;
    // error_reporter = &micro_error_reporter;

    // Load release model (model loader should already be initialized)
    const unsigned char *model_data = get_release_model_data();

    release_model = tflite::GetModel(model_data);
    if (release_model->version() != TFLITE_SCHEMA_VERSION)
    {
        MicroPrintf(
            "Model provided is schema version %d not equal "
            "to supported version %d.",
            release_model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }
    // CNN
    static tflite::MicroMutableOpResolver<6> micro_op_resolver;
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddSoftmax();
    static tflite::MicroInterpreter static_interpreter(release_model, micro_op_resolver, release_tensor_arena,
                                                       release_kTensorArenaSize);

    release_interpreter = &static_interpreter;

    MicroPrintf("Allocate Tensors started.\n");

    TfLiteStatus allocate_status = release_interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk)
    {
        TF_LITE_REPORT_ERROR(release_error_reporter, "Failed to Allocate Tensors");
        return;
    }
    MicroPrintf("Allocate Tensors Successfully\n");
    TfLiteTensor *model_input = release_interpreter->input(0);
    int dimension_size = model_input->dims->size;
    MicroPrintf("dims->size = %d\n", dimension_size);
    MicroPrintf("model_input->dims->data[0] = %d\n", model_input->dims->data[0]);
    MicroPrintf("model_input->dims->data[1] = %d\n", model_input->dims->data[1]);
    MicroPrintf("model_input->dims->data[2] = kChannelReleaseNumber = %d\n", model_input->dims->data[2]);
    MicroPrintf("model_input->type = %d\n", model_input->type);
    int input_bytes = model_input->bytes;
    MicroPrintf("model input bytes = %d\n", input_bytes);
    int output_bytes = release_interpreter->output(0)->bytes;
    MicroPrintf("model output bytes = %d\n", output_bytes);
    int input_length = 0;
    int output_length = 0;
#if kModelDataType == kTfLiteFloat32
    input_length = model_input->bytes / sizeof(float);
#elif kModelDataType == kTfLiteInt16
    input_length = model_input->bytes / sizeof(int16_t);
#endif

#if kModelDataType == kTfLiteFloat32
    output_length = release_interpreter->output(0)->bytes / sizeof(float);
#elif kModelDataType == kTfLiteInt16
    output_length = release_interpreter->output(0)->bytes / sizeof(int16_t);
#endif

    model_input = release_interpreter->input(0);
    if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
        (model_input->dims->data[1] != release_numSamples) ||
        (model_input->dims->data[2] != kChannelReleaseNumber) ||
        (model_input->type != kModelDataType))
    {
        TF_LITE_REPORT_ERROR(release_error_reporter, "Bad input tensor parameters in model");
        return;
    }

    if (input_length != release_numSamples * kChannelReleaseNumber)
    {
        TF_LITE_REPORT_ERROR(release_error_reporter, "error input length");
        return;
    }

    if (output_length != kGestureCount)
    {
        TF_LITE_REPORT_ERROR(release_error_reporter, "Bad output tensor parameters in model");
        return;
    }

    // extern void gesture_detect_test(tflite::MicroInterpreter * interpreter);
    // gesture_detect_test(interpreter);
    // 在模型初始化後，獲取並顯示量化參數
    float input_scale = release_interpreter->input(0)->params.scale;
    int input_zero_point = release_interpreter->input(0)->params.zero_point;
    MicroPrintf("Quantization params: scale=%f, zero_point=%d\n", input_scale, input_zero_point);
}

int recognize_gesture_tap(float (*matrix)[kChannelNumber])
{
    TfLiteTensor *model_input = interpreter->input(0);
    for (uint8_t i = 0; i < numSamples; i++)
    {
        for (uint8_t j = 0; j < kChannelNumber; j++)
        {
            float temp = (float)(*(matrix[i] + j));
            model_input->data.f[i * kChannelNumber + j] = temp;
        }
        // MicroPrintf("matrix[%d](x,y,z) = (%f, %f, %f)\n", i, model_input->data.f[i * kChannelNumber],
        //             model_input->data.f[i * kChannelNumber + 1], model_input->data.f[i * kChannelNumber + 2]);
    }
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk)
    {
        return 404;
    }

    // int gesture_index = PredictGesture(interpreter->output(0)->data.f);
    // return gesture_index;
    return interpreter->output(0)->data.f[1] * 100;
}

int recognize_gesture_release(float (*matrix)[kChannelReleaseNumber])
{
    TfLiteTensor *model_input = release_interpreter->input(0);
    for (uint8_t i = 0; i < release_numSamples; i++)
    {
        for (uint8_t j = 0; j < kChannelReleaseNumber; j++)
        {
            float temp = (float)(*(matrix[i] + j));
#if kModelDataType == kTfLiteFloat32
            model_input->data.f[i * kChannelReleaseNumber + j] = temp;
#elif kModelDataType == kTfLiteInt16
#if (kChannelReleaseNumber == 4)
            if (j == 3)
            {
                model_input->data.i16[i * kChannelNumber + j] = (int16_t)(temp); // 量化到[-32768, 32767]
            }
            else
#endif
            {
                model_input->data.i16[i * kChannelNumber + j] = (int16_t)(temp / 39.2 * 32767.0f); // 量化到[-32768, 32767]
            }
#endif
        }
    }
    TfLiteStatus invoke_status = release_interpreter->Invoke();
    if (invoke_status != kTfLiteOk)
    {
        return 404;
    }
    // #if kModelDataType == kTfLiteFloat32
    //     MicroPrintf("unknown gesture output[0] = %0.3f\n", release_interpreter->output(0)->data.f[0]);
    //     MicroPrintf("release gesture output[1] = %0.3f\n", release_interpreter->output(0)->data.f[1]);
    //     int gesture_index = PredictGesture(release_interpreter->output(0)->data.f);
    // #elif kModelDataType == kTfLiteInt16
    //     MicroPrintf("unknown gesture output[0] = %d\n", release_interpreter->output(0)->data.i16[0]);
    //     MicroPrintf("release gesture output[1] = %d\n", release_interpreter->output(0)->data.i16[1]);
    //     int gesture_index = PredictQuatizedGesture(release_interpreter->output(0)->data.i16);
    // #endif

    // return gesture_index;
    int gesture_index = PredictGesture(release_interpreter->output(0)->data.f);
    return gesture_index;
}
