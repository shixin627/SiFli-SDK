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
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "grab_micro_features_data.h"
#include "unknown_micro_features_data.h"

static void feed_data_and_test(tflite::MicroInterpreter *interpreter, const float *features_data)
{
	// Obtain a pointer to the model's input tensor
	TfLiteTensor *input = interpreter->input(0);
	MicroPrintf("Feeding features data %d\n", input->bytes);
	for (size_t i = 0; i < (input->bytes / sizeof(float)); ++i)
	{
		input->data.f[i] = features_data[i];
	}

	// Run the model on this input and check that it succeeds
	TfLiteStatus invoke_status = interpreter->Invoke();
	if (invoke_status != kTfLiteOk)
	{
		MicroPrintf("Invoke failed\n");
	}
	// Obtain a pointer to the output tensor and make sure it has the
	// properties we expect.
	TfLiteTensor *output = interpreter->output(0);

	// There are four possible classes in the output, each with a score.
	const int kUnknownIndex = 0;
	const int kGrabIndex = 1;

	float unknown_score = output->data.f[kUnknownIndex];
	float grab_score = output->data.f[kGrabIndex];
	MicroPrintf("Output scores Unknown=%f, Grab=%f\n", unknown_score, grab_score);
}

void gesture_detect_test(tflite::MicroInterpreter *interpreter)
{
	MicroPrintf(" <---------------------- Start gesture_detect_test ---------------------->\n");

	// Provide an input value
	MicroPrintf("testing unknown gesture\n");
	const float *unknown_features_data = g_unknown_micro_f9643d42_nohash_4_data;
	feed_data_and_test(interpreter, unknown_features_data);
	MicroPrintf("testing grab gesture\n");
	const float *grab_features_data = g_grab_micro_f9643d42_nohash_4_data;
	feed_data_and_test(interpreter, grab_features_data);

	MicroPrintf(" <---------------------- End gesture_detect_test ---------------------->\n");
}
