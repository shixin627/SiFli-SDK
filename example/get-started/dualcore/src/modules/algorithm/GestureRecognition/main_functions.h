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
#ifndef TENSORFLOW_LITE_MICRO_MOTION_WATCH_MAIN_FUNCTIONS_H_
#define TENSORFLOW_LITE_MICRO_MOTION_WATCH_MAIN_FUNCTIONS_H_

#include <stdint.h>
#include "constants.h"

#ifdef __cplusplus
extern "C"
{
#endif
    // Initializes all data needed for the example. The name is important, and needs
    // to be setup() for Arduino compatibility.
    void init_gesture_recognition_model(void);
    int recognize_gesture_tap(float (*matrix)[kChannelNumber]);
    void init_gesture_recognition_release_model(void);
    int recognize_gesture_release(float (*matrix)[kChannelReleaseNumber]);
#ifdef __cplusplus
}
#endif

#endif // TENSORFLOW_LITE_MICRO_MOTION_WATCH_MAIN_FUNCTIONS_H_
