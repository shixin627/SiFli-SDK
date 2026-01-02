/* Copyright 2022 The TensorFlow Authors. All Rights Reserved.

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

#include "tensorflow/lite/micro/kernels/conv.h"

#include "arm_nnfunctions.h"
#include "tensorflow/lite/c/builtin_op_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/kernels/internal/common.h"
#include "tensorflow/lite/kernels/internal/quantization_util.h"
#include "tensorflow/lite/kernels/internal/reference/conv.h"
#include "tensorflow/lite/kernels/internal/reference/integer_ops/conv.h"
#include "tensorflow/lite/kernels/internal/tensor_ctypes.h"
#include "tensorflow/lite/kernels/kernel_util.h"
#include "tensorflow/lite/kernels/padding.h"
#include "tensorflow/lite/micro/kernels/kernel_util.h"
#include "tensorflow/lite/micro/micro_log.h"

// #define ENABLE_CONV_NN_ACC 1
namespace tflite {
namespace {

struct OpData {
  OpDataConv reference_op_data;

  // Index to buffer for optimizations if applicable.
  int buffer_idx;
};

void* Init(TfLiteContext* context, const char* buffer, size_t length) {
  TFLITE_DCHECK(context->AllocatePersistentBuffer != nullptr);
  return context->AllocatePersistentBuffer(context, sizeof(OpData));
}

TfLiteStatus Prepare(TfLiteContext* context, TfLiteNode* node) {
  TFLITE_DCHECK(node->user_data != nullptr);
  TFLITE_DCHECK(node->builtin_data != nullptr);
  int32_t buf_size = 0;
  const auto& params =
      *(static_cast<const TfLiteConvParams*>(node->builtin_data));
  OpData* data = static_cast<OpData*>(node->user_data);

  MicroContext* micro_context = GetMicroContext(context);

  TfLiteTensor* input =
      micro_context->AllocateTempInputTensor(node, kConvInputTensor);
  TF_LITE_ENSURE(context, input != nullptr);
  TfLiteTensor* filter =
      micro_context->AllocateTempInputTensor(node, kConvWeightsTensor);
  TF_LITE_ENSURE(context, filter != nullptr);
  TfLiteTensor* output =
      micro_context->AllocateTempOutputTensor(node, kConvOutputTensor);
  TF_LITE_ENSURE(context, output != nullptr);

  RuntimeShape input_shape = GetTensorShape(input);
  RuntimeShape output_shape = GetTensorShape(output);

  // Initialize cmsis_nn input dimensions
  cmsis_nn_dims input_dims;
  input_dims.n = MatchingDim(input_shape, 0, output_shape, 0);
  input_dims.h = input->dims->data[1];
  input_dims.w = input->dims->data[2];
  input_dims.c = input_shape.Dims(3);

  // Initialize cmsis_nn filter dimensions
  cmsis_nn_dims filter_dims;
  filter_dims.n = output_shape.Dims(3);
  filter_dims.h = filter->dims->data[1];
  filter_dims.w = filter->dims->data[2];
  filter_dims.c = input_dims.c;

  // Initialize cmsis_nn output dimensions
  cmsis_nn_dims output_dims;
  output_dims.n = input_dims.n;
  output_dims.h = output->dims->data[1];
  output_dims.w = output->dims->data[2];
  output_dims.c = output_shape.Dims(3);

  if (filter->type == kTfLiteInt4) {
    int filter_size =
        RuntimeShape(filter->dims->size,
                     reinterpret_cast<const int32_t*>(filter->dims->data))
            .FlatSize();
    context->RequestScratchBufferInArena(
        context, filter_size, &data->reference_op_data.filter_buffer_index);
  }

  if (input->type == kTfLiteInt8 || input->type == kTfLiteInt16) {
    const int num_channels = filter->dims->data[kConvQuantizedDimension];
    data->reference_op_data.per_channel_output_multiplier =
        static_cast<int32_t*>(context->AllocatePersistentBuffer(
            context, num_channels * sizeof(int32_t)));
    data->reference_op_data.per_channel_output_shift =
        static_cast<int32_t*>(context->AllocatePersistentBuffer(
            context, num_channels * sizeof(int32_t)));
  }

  TF_LITE_ENSURE_STATUS(CalculateOpDataConv(
      context, node, params, input_dims.w, input_dims.h, filter_dims.w,
      filter_dims.h, output_dims.w, output_dims.h, input->type,
      &data->reference_op_data));

  if (input->type == kTfLiteInt8 || input->type == kTfLiteInt16) {
    // Initialize cmsis_nn convolution parameters
    cmsis_nn_conv_params conv_params;
    conv_params.input_offset = -input->params.zero_point;
    conv_params.output_offset = output->params.zero_point;
    conv_params.stride.h = params.stride_height;
    conv_params.stride.w = params.stride_width;
    conv_params.dilation.h = params.dilation_height_factor;
    conv_params.dilation.w = params.dilation_width_factor;
    conv_params.padding.h = data->reference_op_data.padding.height;
    conv_params.padding.w = data->reference_op_data.padding.width;
    conv_params.activation.min = data->reference_op_data.output_activation_min;
    conv_params.activation.max = data->reference_op_data.output_activation_max;

    if (input->type == kTfLiteInt8) {
#if defined(ENABLE_CONV_NN_ACC)
      // 檢查是否是小輸入通道數的常規卷積，可以使用 arm_convolve_HWC_q7_basic_nonsquare
      if (input_dims.c <= 16) {
        // 為 HWC 卷積計算所需的緩衝區大小
        int buffer_size_A = 2 * input_dims.c * filter_dims.w * filter_dims.h * sizeof(q15_t);
        // 緩衝區 B 的大小可以根據實際需要調整
        int buffer_size_B = input_dims.c * filter_dims.w * filter_dims.h * sizeof(q7_t);
        int basic_conv_buf_size = buffer_size_A + buffer_size_B;
        
        // 確保緩衝區大小足夠
        buf_size = std::max(buf_size, basic_conv_buf_size);
      } else 
#endif
      {
        // 原有代碼：獲取 arm_convolve_wrapper_s8 需要的緩衝區大小
        buf_size = arm_convolve_wrapper_s8_get_buffer_size(
            &conv_params, &input_dims, &filter_dims, &output_dims);
      }
    } else if (input->type == kTfLiteInt16) {
      TF_LITE_ENSURE_EQ(context, input->params.zero_point, 0);
      TF_LITE_ENSURE_EQ(context, output->params.zero_point, 0);
      buf_size = arm_convolve_wrapper_s16_get_buffer_size(
          &conv_params, &input_dims, &filter_dims, &output_dims);
    }

    if (buf_size > 0) {
      TF_LITE_ENSURE_STATUS(context->RequestScratchBufferInArena(
          context, buf_size, &data->buffer_idx));
    } else {
      data->buffer_idx = -1;
    }
  }

  micro_context->DeallocateTempTfLiteTensor(output);
  micro_context->DeallocateTempTfLiteTensor(input);
  micro_context->DeallocateTempTfLiteTensor(filter);

  return kTfLiteOk;
}

TfLiteStatus EvalQuantizedPerChannel(TfLiteContext* context, TfLiteNode* node,
                                     const TfLiteConvParams& params,
                                     const OpData& data,
                                     const TfLiteEvalTensor* input,
                                     const TfLiteEvalTensor* filter,
                                     const TfLiteEvalTensor* bias,
                                     TfLiteEvalTensor* output) {
  cmsis_nn_conv_params conv_params;
  conv_params.dilation.h = params.dilation_height_factor;
  conv_params.dilation.w = params.dilation_width_factor;

  // Initialize cmsis_nn convolution parameters
  conv_params.input_offset = -data.reference_op_data.input_zero_point;
  conv_params.output_offset = data.reference_op_data.output_zero_point;
  conv_params.stride.h = params.stride_height;
  conv_params.stride.w = params.stride_width;
  conv_params.padding.h = data.reference_op_data.padding.height;
  conv_params.padding.w = data.reference_op_data.padding.width;
  conv_params.activation.min = data.reference_op_data.output_activation_min;
  conv_params.activation.max = data.reference_op_data.output_activation_max;

  // Initialize cmsis_nn per channel quantization parameters
  cmsis_nn_per_channel_quant_params quant_params;
  quant_params.multiplier = const_cast<int32_t*>(
      data.reference_op_data.per_channel_output_multiplier);
  quant_params.shift =
      const_cast<int32_t*>(data.reference_op_data.per_channel_output_shift);

  RuntimeShape filter_shape = tflite::micro::GetTensorShape(filter);
  RuntimeShape input_shape = tflite::micro::GetTensorShape(input);
  RuntimeShape output_shape = tflite::micro::GetTensorShape(output);
  RuntimeShape bias_shape = tflite::micro::GetTensorShape(bias);

  // Consistency check.
  TFLITE_DCHECK_LE(conv_params.activation.min, conv_params.activation.max);
  TFLITE_DCHECK_EQ(input_shape.DimensionsCount(), 4);
  TFLITE_DCHECK_EQ(filter_shape.DimensionsCount(), 4);
  TFLITE_DCHECK_EQ(output_shape.DimensionsCount(), 4);
  const int batch_size = MatchingDim(input_shape, 0, output_shape, 0);
  const int input_depth = MatchingDim(input_shape, 3, filter_shape, 3);
  const int output_depth = MatchingDim(filter_shape, 0, output_shape, 3);
  if (tflite::micro::GetOptionalTensorData<int8_t>(bias)) {
    TFLITE_DCHECK_EQ(bias_shape.FlatSize(), output_depth);
  }
  // Initialize cmsis_nn dimensions
  // Input
  cmsis_nn_dims input_dims;
  input_dims.n = batch_size;
  input_dims.h = input_shape.Dims(1);
  input_dims.w = input_shape.Dims(2);
  input_dims.c = input_depth;

  // Filter
  cmsis_nn_dims filter_dims;
  filter_dims.n = output_depth;
  filter_dims.h = filter_shape.Dims(1);
  filter_dims.w = filter_shape.Dims(2);
  filter_dims.c = input_depth;

  // Bias
  cmsis_nn_dims bias_dims;
  bias_dims.n = 1;
  bias_dims.h = 1;
  bias_dims.w = 1;
  bias_dims.c = output_depth;

  // Output
  cmsis_nn_dims output_dims;
  output_dims.n = batch_size;
  output_dims.h = output_shape.Dims(1);
  output_dims.w = output_shape.Dims(2);
  output_dims.c = output_depth;

  // Initialize cmsis_nn context
  cmsis_nn_context ctx;
  ctx.buf = nullptr;
  ctx.size = 0;

  if (data.buffer_idx > -1) {
    ctx.buf = context->GetScratchBuffer(context, data.buffer_idx);
    // Note: ctx.size is currently not used in cmsis_nn.
    // The buffer should be allocated in the Prepare function through
    // arm_convolve_wrapper_s8_get_buffer_size
  }
#if defined(ENABLE_CONV_NN_ACC)
  // 檢查是否為 1x1 卷積且符合優化條件
  if (filter_dims.w == 1 && filter_dims.h == 1 &&
    conv_params.padding.w == 0 && conv_params.padding.h == 0 &&
    conv_params.stride.w == 1 && conv_params.stride.h == 1 &&
    input_depth % 4 == 0 && output_depth % 2 == 0) {

    const q7_t *Im_in = tflite::micro::GetTensorData<int8_t>(input);
    const q7_t *wt = tflite::micro::GetTensorData<int8_t>(filter);
    const q7_t *bias_data = tflite::micro::GetOptionalTensorData<int8_t>(bias);
    q7_t *Im_out = tflite::micro::GetTensorData<int8_t>(output);
    // 準備緩衝區 - 需要從上下文獲取
    q15_t* bufferA = nullptr;
    q7_t* bufferB = nullptr;
    
    if (ctx.buf != nullptr) {
      // 假設緩衝區已經在 Prepare 中分配
      bufferA = static_cast<q15_t*>(ctx.buf);
      // bufferB 可以分配在 bufferA 後面
      bufferB = reinterpret_cast<q7_t*>(bufferA + (input_depth * 2));
    }
    
    // 調用加速函數
    arm_status status = arm_convolve_1x1_HWC_q7_fast_nonsquare(
        Im_in,
        input_dims.w,
        input_dims.h,
        input_depth,
        wt,
        output_depth,
        1,  // dim_kernel_x
        1,  // dim_kernel_y
        0,  // padding_x
        0,  // padding_y
        1,  // stride_x
        1,  // stride_y
        bias_data,
        0, // bias_shift (需調整)
        data.reference_op_data.output_shift,           // out_shift (需調整)
        Im_out,
        output_dims.w,
        output_dims.h,
        bufferA,
        bufferB);
        
    if (status == ARM_MATH_SUCCESS) {
      return kTfLiteOk;
    }
  }
  // 檢查是否可以應用一般卷積的優化
  else if (input_depth <= 16) { // 限制輸入通道數量以確保緩衝區足夠
    const q7_t *Im_in = tflite::micro::GetTensorData<int8_t>(input);
    const q7_t *wt = tflite::micro::GetTensorData<int8_t>(filter);
    const q7_t *bias_data = tflite::micro::GetOptionalTensorData<int8_t>(bias);
    q7_t *Im_out = tflite::micro::GetTensorData<int8_t>(output);
    
    // 準備緩衝區
    q15_t* bufferA = nullptr;
    q7_t* bufferB = nullptr;
    
    if (ctx.buf != nullptr) {
      // 為 HWC 卷積分配足夠的緩衝區空間
      bufferA = static_cast<q15_t*>(ctx.buf);
      // 為輸出緩沖區計算位置，確保不與 bufferA 重疊
      int bufferA_size = 2 * input_depth * filter_dims.w * filter_dims.h;
      bufferB = reinterpret_cast<q7_t*>(bufferA + bufferA_size);
    }
    
    // 計算量化參數
    int32_t effective_bias_shift = 0;
    int32_t effective_out_shift = 0;
    
    // 由於 CMSIS-NN 和 TFLite 的量化參數有差異，這裡需要進行轉換
    // 簡化示例 - 您可能需要更精確的轉換
    effective_out_shift = -data.reference_op_data.output_shift; // 注意：TFLite 和 CMSIS-NN 的移位方向可能相反
    
    // 調用 CMSIS-NN 基本卷積函數
    arm_status status = arm_convolve_HWC_q7_basic_nonsquare(
        Im_in,
        input_dims.w,
        input_dims.h,
        input_depth,
        wt,
        output_depth,
        filter_dims.w,
        filter_dims.h,
        conv_params.padding.w,
        conv_params.padding.h,
        conv_params.stride.w,
        conv_params.stride.h,
        bias_data,
        effective_bias_shift,
        effective_out_shift,
        Im_out,
        output_dims.w,
        output_dims.h,
        bufferA,
        bufferB);
    if (status == ARM_MATH_SUCCESS) {
      // 應用激活函數（如果需要）
      // 注意：CMSIS-NN 的卷積函數可能沒有應用激活，這取決於具體實現
      int num_elements = output_dims.n * output_dims.h * output_dims.w * output_dims.c;
      for (int i = 0; i < num_elements; i++) {
        Im_out[i] = std::max<int8_t>(conv_params.activation.min, 
                    std::min<int8_t>(conv_params.activation.max, Im_out[i]));
      }
      
      return kTfLiteOk;
    }
  }
#endif
  // arm_convolve_wrapper_s8 dispatches the optimized kernel accordingly with
  // the parameters passed
  TFLITE_DCHECK_EQ(
      arm_convolve_wrapper_s8(
          &ctx, &conv_params, &quant_params, &input_dims,
          tflite::micro::GetTensorData<int8_t>(input), &filter_dims,
          tflite::micro::GetTensorData<int8_t>(filter), &bias_dims,
          tflite::micro::GetOptionalTensorData<int32_t>(bias), &output_dims,
          tflite::micro::GetTensorData<int8_t>(output)),
      ARM_CMSIS_NN_SUCCESS);

  return kTfLiteOk;
}

TfLiteStatus EvalQuantizedPerChannel16x8(
    TfLiteContext* context, TfLiteNode* node, const TfLiteConvParams& params,
    const OpData& data, const TfLiteEvalTensor* input,
    const TfLiteEvalTensor* filter, const TfLiteEvalTensor* bias,
    TfLiteEvalTensor* output) {
  cmsis_nn_conv_params conv_params;
  conv_params.dilation.h = params.dilation_height_factor;
  conv_params.dilation.w = params.dilation_width_factor;

  // Initialize cmsis_nn convolution parameters
  conv_params.input_offset = -data.reference_op_data.input_zero_point;
  conv_params.output_offset = data.reference_op_data.output_zero_point;
  conv_params.stride.h = params.stride_height;
  conv_params.stride.w = params.stride_width;
  conv_params.padding.h = data.reference_op_data.padding.height;
  conv_params.padding.w = data.reference_op_data.padding.width;
  conv_params.activation.min = data.reference_op_data.output_activation_min;
  conv_params.activation.max = data.reference_op_data.output_activation_max;

  // Initialize cmsis_nn per channel quantization parameters
  cmsis_nn_per_channel_quant_params quant_params;
  quant_params.multiplier = const_cast<int32_t*>(
      data.reference_op_data.per_channel_output_multiplier);
  quant_params.shift =
      const_cast<int32_t*>(data.reference_op_data.per_channel_output_shift);

  RuntimeShape filter_shape = tflite::micro::GetTensorShape(filter);
  RuntimeShape input_shape = tflite::micro::GetTensorShape(input);
  RuntimeShape output_shape = tflite::micro::GetTensorShape(output);
  RuntimeShape bias_shape = tflite::micro::GetTensorShape(bias);

  // Consistency check.
  TFLITE_DCHECK_LE(conv_params.activation.min, conv_params.activation.max);
  TFLITE_DCHECK_EQ(input_shape.DimensionsCount(), 4);
  TFLITE_DCHECK_EQ(filter_shape.DimensionsCount(), 4);
  TFLITE_DCHECK_EQ(output_shape.DimensionsCount(), 4);
  const int batch_size = MatchingDim(input_shape, 0, output_shape, 0);
  const int input_depth = MatchingDim(input_shape, 3, filter_shape, 3);
  const int output_depth = MatchingDim(filter_shape, 0, output_shape, 3);
  if (tflite::micro::GetOptionalTensorData<int8_t>(bias)) {
    TFLITE_DCHECK_EQ(bias_shape.FlatSize(), output_depth);
  }
  // Initialize cmsis_nn dimensions
  // Input
  cmsis_nn_dims input_dims;
  input_dims.n = batch_size;
  input_dims.h = input_shape.Dims(1);
  input_dims.w = input_shape.Dims(2);
  input_dims.c = input_depth;

  // Filter
  cmsis_nn_dims filter_dims;
  filter_dims.n = output_depth;
  filter_dims.h = filter_shape.Dims(1);
  filter_dims.w = filter_shape.Dims(2);
  filter_dims.c = input_depth;

  // Bias
  cmsis_nn_dims bias_dims;
  bias_dims.n = 1;
  bias_dims.h = 1;
  bias_dims.w = 1;
  bias_dims.c = output_depth;

  // Output
  cmsis_nn_dims output_dims;
  output_dims.n = batch_size;
  output_dims.h = output_shape.Dims(1);
  output_dims.w = output_shape.Dims(2);
  output_dims.c = output_depth;

  // Initialize cmsis_nn context
  cmsis_nn_context ctx;
  ctx.buf = nullptr;
  ctx.size = 0;

  if (data.buffer_idx > -1) {
    ctx.buf = context->GetScratchBuffer(context, data.buffer_idx);
    // Note: ctx.size is currently not used in cmsis_nn.
    // The buffer should be allocated in the Prepare function through
    // arm_convolve_wrapper_s8_get_buffer_size
  }

  MicroPrintf("[Conv]EvalQuantizedPerChannel16x8 input type = %s, filter type = %s, output type = %s\n",
    TfLiteTypeGetName(input->type), TfLiteTypeGetName(filter->type),
    TfLiteTypeGetName(output->type));

  TFLITE_DCHECK_EQ(
      arm_convolve_wrapper_s16(
          &ctx, &conv_params, &quant_params, &input_dims,
          tflite::micro::GetTensorData<int16_t>(input), &filter_dims,
          tflite::micro::GetTensorData<int8_t>(filter), &bias_dims,
          tflite::micro::GetOptionalTensorData<int64_t>(bias), &output_dims,
          tflite::micro::GetTensorData<int16_t>(output)),
      ARM_CMSIS_NN_SUCCESS);

  return kTfLiteOk;
}

TfLiteStatus EvalInt8(TfLiteContext* context, TfLiteNode* node) {
  const TfLiteEvalTensor* input =
      tflite::micro::GetEvalInput(context, node, kConvInputTensor);
  const TfLiteEvalTensor* filter =
      tflite::micro::GetEvalInput(context, node, kConvWeightsTensor);
  const TfLiteEvalTensor* bias =
      (NumInputs(node) == 3)
          ? tflite::micro::GetEvalInput(context, node, kConvBiasTensor)
          : nullptr;
  TfLiteEvalTensor* output =
      tflite::micro::GetEvalOutput(context, node, kConvOutputTensor);

  TFLITE_DCHECK(node->builtin_data != nullptr);
  const auto& params =
      *(reinterpret_cast<TfLiteConvParams*>(node->builtin_data));
  TFLITE_DCHECK(node->user_data != nullptr);
  const OpData& data = *(static_cast<const OpData*>(node->user_data));
  TfLiteEvalTensor filter_int8 = tflite::micro::MakeUnpackedInt4Tensor(
      context, data.reference_op_data.filter_buffer_index, filter);

  return EvalQuantizedPerChannel(context, node, params, data, input,
                                 &filter_int8, bias, output);
}

TfLiteStatus EvalInt16x8(TfLiteContext* context, TfLiteNode* node) {
  const TfLiteEvalTensor* input =
      tflite::micro::GetEvalInput(context, node, kConvInputTensor);
  const TfLiteEvalTensor* filter =
      tflite::micro::GetEvalInput(context, node, kConvWeightsTensor);
  const TfLiteEvalTensor* bias =
      (NumInputs(node) == 3)
          ? tflite::micro::GetEvalInput(context, node, kConvBiasTensor)
          : nullptr;
  TfLiteEvalTensor* output =
      tflite::micro::GetEvalOutput(context, node, kConvOutputTensor);

  TFLITE_DCHECK(node->builtin_data != nullptr);
  const auto& params =
      *(reinterpret_cast<TfLiteConvParams*>(node->builtin_data));
  TFLITE_DCHECK(node->user_data != nullptr);
  const OpData& data = *(static_cast<const OpData*>(node->user_data));
  return EvalQuantizedPerChannel16x8(context, node, params, data, input, filter,
                                     bias, output);
}

TfLiteStatus Eval(TfLiteContext* context, TfLiteNode* node) {
  const TfLiteEvalTensor* input =
      tflite::micro::GetEvalInput(context, node, kConvInputTensor);
  const TfLiteEvalTensor* filter =
      tflite::micro::GetEvalInput(context, node, kConvWeightsTensor);
  const TfLiteEvalTensor* bias =
      (NumInputs(node) == 3)
          ? tflite::micro::GetEvalInput(context, node, kConvBiasTensor)
          : nullptr;
  TfLiteEvalTensor* output =
      tflite::micro::GetEvalOutput(context, node, kConvOutputTensor);

  TFLITE_DCHECK(node->builtin_data != nullptr);
  const auto& params =
      *(reinterpret_cast<TfLiteConvParams*>(node->builtin_data));
  TFLITE_DCHECK(node->user_data != nullptr);
  const OpData& data = *(static_cast<const OpData*>(node->user_data));

  TF_LITE_ENSURE_EQ(context, input->type, output->type);
  TF_LITE_ENSURE_MSG(
      context,
      input->type == filter->type ||
          (input->type == kTfLiteInt16 && filter->type == kTfLiteInt8) ||
          (input->type == kTfLiteInt8 && filter->type == kTfLiteInt4),
      "Hybrid models are not supported on TFLite Micro.");

  TfLiteEvalTensor filter_int8 = tflite::micro::MakeUnpackedInt4Tensor(
      context, data.reference_op_data.filter_buffer_index, filter);

  switch (input->type) {  // Already know in/out types are same.
    case kTfLiteFloat32: {
      tflite::reference_ops::Conv(
          ConvParamsFloat(params, data.reference_op_data),
          tflite::micro::GetTensorShape(input),
          tflite::micro::GetTensorData<float>(input),
          tflite::micro::GetTensorShape(filter),
          tflite::micro::GetTensorData<float>(filter),
          tflite::micro::GetTensorShape(bias),
          tflite::micro::GetOptionalTensorData<float>(bias),
          tflite::micro::GetTensorShape(output),
          tflite::micro::GetTensorData<float>(output),
          tflite::micro::GetTensorShape(nullptr), nullptr);
      break;
    }
    case kTfLiteInt8:
      switch (filter_int8.type) {
        case kTfLiteInt8: {
          return EvalQuantizedPerChannel(context, node, params, data, input,
                                         &filter_int8, bias, output);
        }

        default: {
          MicroPrintf("Filter type %s (%d) not supported.",
                      TfLiteTypeGetName(filter->type), filter->type);
          return kTfLiteError;
        }
      }

      break;
    case kTfLiteInt16:
      return EvalQuantizedPerChannel16x8(context, node, params, data, input,
                                         filter, bias, output);
      break;
    default:
      MicroPrintf("Type %s (%d) not supported.", TfLiteTypeGetName(input->type),
                  input->type);
      return kTfLiteError;
  }
  return kTfLiteOk;
}

}  // namespace

TfLiteRegistration Register_CONV_2D() {
  return tflite::micro::RegisterOp(Init, Prepare, Eval);
}

TfLiteRegistration Register_CONV_2D_INT8() {
  return tflite::micro::RegisterOp(Init, Prepare, EvalInt8);
}

TfLiteRegistration Register_CONV_2D_INT16() {
  return tflite::micro::RegisterOp(Init, Prepare, EvalInt16x8);
}

}  // namespace tflite
