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

#ifndef TENSORFLOW_LITE_MICRO_EXAMPLES_MAGIC_WAND_GESTURE_PREDICTOR_H_
#define TENSORFLOW_LITE_MICRO_EXAMPLES_MAGIC_WAND_GESTURE_PREDICTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif

	extern int PredictGesture(float *output);
	extern int PredictQuatizedGesture(int16_t *output);

	/* Tap 的信心門檻(0..1)。低於它,模型即使把 tap 選成 argmax 也回
	   kUnknownGesture。預設值與由來見 gesture_predictor.cc。 */
	void gesture_set_tap_confidence(float thr);
	float gesture_get_tap_confidence(void);

#ifdef __cplusplus
}
#endif

#endif  // TENSORFLOW_LITE_MICRO_EXAMPLES_MAGIC_WAND_GESTURE_PREDICTOR_H_