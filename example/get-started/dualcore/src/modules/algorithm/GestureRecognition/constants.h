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

#ifndef TENSORFLOW_LITE_MICRO_EXAMPLES_MAGIC_WAND_CONSTANTS_H_
#define TENSORFLOW_LITE_MICRO_EXAMPLES_MAGIC_WAND_CONSTANTS_H_

#define kChannelNumber 3
#define kChannelReleaseNumber 3
#define kModelDataType kTfLiteFloat32

// The expected accelerometer data sample frequency
#define kTargetHz 100.0f

// What gestures are supported.
#define kGestureCount 2

#define kReleaseGesture 1
#define kGrabGesture 4
#define kDoubleTapGesture 3
#define kHoldGesture 2
#define kTapGesture 1
#define kNoGesture 0

// These control the sensitivity of the detection algorithm. If you're seeing
// too many false positives or not enough true positives, you can try tweaking
// these thresholds. Often, increasing the size of the training set will give
// more robust results though, so consider retraining if you are seeing poor
// predictions.
#define kDetectionThreshold 0.8f
#define kPredictionHistoryLength 5
#define kPredictionSuppressionDuration 25

#endif // TENSORFLOW_LITE_MICRO_EXAMPLES_MAGIC_WAND_CONSTANTS_H_