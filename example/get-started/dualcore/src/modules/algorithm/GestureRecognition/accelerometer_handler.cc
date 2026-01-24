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

#include "accelerometer_handler.h"

static const int max_sample_amount = 15;
static const int time_steps = 15;
static int sample_count = 0;
TfLiteStatus SetupAccelerometer() { return kTfLiteOk; }

static void update_array(float *input, int start_index, float *new_values)
{
	for (int i = 0; i < 3; i++)
	{
		input[start_index + i] = new_values[i];
	}
}

static void move_time_steps(float *input, uint8_t time_steps)
{
	for (int i = 0; i < (max_sample_amount - time_steps); i++)
	{
		input[i * 3] = input[(time_steps + i) * 3];
		input[i * 3 + 1] = input[(time_steps + i) * 3 + 1];
		input[i * 3 + 2] = input[(time_steps + i) * 3 + 2];
	}
}

bool ReadAccelerometer(float *input, float *new_values)
{
	if (sample_count == max_sample_amount)
	{
		move_time_steps(input, time_steps);
		sample_count -= time_steps;
	}
	if (sample_count < max_sample_amount)
	{
		update_array(input, sample_count * 3, new_values);
		sample_count++;
		if (sample_count == max_sample_amount)
		{
			return true;
		}
	}
	return false;
}
