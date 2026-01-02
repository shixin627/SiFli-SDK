/* Copyright 2018 The TensorFlow Authors. All Rights Reserved.

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

// Implements debug logging for numbers by converting them into strings and then
// calling the main DebugLog(char*) function. These are separated into a
// different file so that platforms can just implement the string output version
// of DebugLog() and then get the numerical variations without requiring any
// more code.

#include "tensorflow/lite/micro/micro_string.h"
#include "tensorflow/Arduino.h"

extern "C" int MicroVsnprintf(char* output, int len, const char* format,
                              va_list args) {
  return rt_vsnprintf(output, len, format, args);
}

extern "C" int MicroSnprintf(char* output, int len, const char* format, ...) {
  va_list args;
  va_start(args, format);
  int bytes_written = MicroVsnprintf(output, len, format, args);
  va_end(args);
  return bytes_written;
}
