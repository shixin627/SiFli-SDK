/* Copyright 2021 The TensorFlow Authors. All Rights Reserved.

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

#include "tensorflow/lite/micro/system_setup.h"

#include <limits>

#include "tensorflow/lite/micro/debug_log.h"

#include "tensorflow/Arduino.h"

// #define DEBUG_SERIAL_OBJECT (Serial)
#ifndef RT_USING_CONSOLE
#define DEBUG_SERIAL_OBJECT(...)
#else
#define DEBUG_SERIAL_OBJECT (rt_kprintf)
#endif

// extern "C" void DebugLog(const char* s) { DEBUG_SERIAL_OBJECT.print(s); }
extern "C" void DebugLog(const char *s) { DEBUG_SERIAL_OBJECT(s); }

namespace tflite
{

  constexpr unsigned long kSerialMaxInitWait = 4000; // milliseconds

  void InitializeTarget()
  {
#ifdef RT_USING_CONSOLE
    // DEBUG_SERIAL_OBJECT.begin();
    // unsigned long start_time = millis();
    unsigned long start_time = rt_tick_get_millisecond();
    while (!DEBUG_SERIAL_OBJECT)
    {
      // allow for Arduino IDE Serial Monitor synchronization
      // if (millis() - start_time > kSerialMaxInitWait)
      if (rt_tick_get_millisecond() - start_time > kSerialMaxInitWait)
      {
        break;
      }
    }
#endif
  }

} // namespace tflite
