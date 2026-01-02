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

#include "tensorflow/lite/micro/kernels/depthwise_conv.h"

#include "arm_nnfunctions.h"
#include "tensorflow/lite/c/builtin_op_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/kernels/internal/common.h"
#include "tensorflow/lite/kernels/internal/quantization_util.h"
#include "tensorflow/lite/kernels/internal/reference/depthwiseconv_float.h"
#include "tensorflow/lite/kernels/internal/reference/integer_ops/depthwise_conv.h"
#include "tensorflow/lite/kernels/internal/tensor_ctypes.h"
#include "tensorflow/lite/kernels/kernel_util.h"
#include "tensorflow/lite/kernels/padding.h"
#include "tensorflow/lite/micro/kernels/conv.h"
#include "tensorflow/lite/micro/kernels/kernel_util.h"
#include "tensorflow/lite/micro/micro_log.h"
// #define ENABLE_DEPTH_CONV_NN_ACC 1
namespace tflite {
namespace {

struct OpData {
  OpDataConv reference_op_data;

  // Index to buffer for optimizations if applicable.
  int buffer_idx;
};

// Always inline for optimal code size.
void PopulateDwConvParams(
    cmsis_nn_dw_conv_params* const dw_conv_params,
    cmsis_nn_per_channel_quant_params* const quant_params,
    cmsis_nn_dims* const input_dims, cmsis_nn_dims* const filter_dims,
    cmsis_nn_dims* const bias_dims, cmsis_nn_dims* const output_dims,
    const TfLiteDepthwiseConvParams& params, const OpData& data,
    const TfLiteEvalTensor* input, const TfLiteEvalTensor* filter,
    const TfLiteEvalTensor* bias, TfLiteEvalTensor* output)
    __attribute__((always_inline));

void* Init(TfLiteContext* context, const char* buffer, size_t length) {
  TFLITE_DCHECK(context->AllocatePersistentBuffer != nullptr);
  return context->AllocatePersistentBuffer(context, sizeof(OpData));
}

TfLiteStatus Prepare(TfLiteContext* context, TfLiteNode* node) {
  TFLITE_DCHECK(node->user_data != nullptr);
  TFLITE_DCHECK(node->builtin_data != nullptr);

  OpData* data = static_cast<OpData*>(node->user_data);
  const auto& params =
      *(reinterpret_cast<TfLiteDepthwiseConvParams*>(node->builtin_data));

  MicroContext* micro_context = GetMicroContext(context);

  TfLiteTensor* input =
      micro_context->AllocateTempInputTensor(node, kDepthwiseConvInputTensor);
  TF_LITE_ENSURE(context, input != nullptr);
  TfLiteTensor* filter =
      micro_context->AllocateTempInputTensor(node, kDepthwiseConvWeightsTensor);
  TF_LITE_ENSURE(context, filter != nullptr);
  TfLiteTensor* output =
      micro_context->AllocateTempOutputTensor(node, kDepthwiseConvOutputTensor);
  TF_LITE_ENSURE(context, output != nullptr);

  const TfLiteType data_type = input->type;
  int input_width = SizeOfDimension(input, 2);
  int input_height = SizeOfDimension(input, 1);
  int filter_width = SizeOfDimension(filter, 2);
  int filter_height = SizeOfDimension(filter, 1);
  int output_width = SizeOfDimension(output, 2);
  int output_height = SizeOfDimension(output, 1);

  if (input->type == kTfLiteInt8 || input->type == kTfLiteInt16) {
    TF_LITE_ENSURE_EQ(context, filter->quantization.type,
                      kTfLiteAffineQuantization);

    if (input->type == kTfLiteInt16) {
      TF_LITE_ENSURE_EQ(context, input->params.zero_point, 0);
      TF_LITE_ENSURE_EQ(context, output->params.zero_point, 0);
    }

    // All per-channel quantized tensors need valid zero point and scale arrays.
    const auto* affine_quantization =
        reinterpret_cast<TfLiteAffineQuantization*>(
            filter->quantization.params);
    TF_LITE_ENSURE(context, affine_quantization);
    TF_LITE_ENSURE(context, affine_quantization->scale);
    TF_LITE_ENSURE(context, affine_quantization->zero_point);
    TF_LITE_ENSURE(
        context, affine_quantization->scale->size == 1 ||
                     affine_quantization->scale->size ==
                         filter->dims->data[kDepthwiseConvQuantizedDimension]);
    TF_LITE_ENSURE_EQ(context, affine_quantization->scale->size,
                      affine_quantization->zero_point->size);

    // Allocate memory for per-channel quantization parameters
    const int num_channels =
        filter->dims->data[kDepthwiseConvQuantizedDimension];

    data->reference_op_data.per_channel_output_multiplier =
        reinterpret_cast<int32_t*>(context->AllocatePersistentBuffer(
            context, num_channels * sizeof(int32_t)));
    data->reference_op_data.per_channel_output_shift =
        reinterpret_cast<int32_t*>(context->AllocatePersistentBuffer(
            context, num_channels * sizeof(int32_t)));
  }

  if (filter->type == kTfLiteInt4) {
    int filter_size =
        RuntimeShape(filter->dims->size,
                     reinterpret_cast<const int32_t*>(filter->dims->data))
            .FlatSize();
    context->RequestScratchBufferInArena(
        context, filter_size, &data->reference_op_data.filter_buffer_index);
  }

  TF_LITE_ENSURE_STATUS(CalculateOpDataDepthwiseConv(
      context, node, params, input_width, input_height, filter_width,
      filter_height, output_width, output_height, data_type,
      &data->reference_op_data));

  if (input->type == kTfLiteInt8) {
    RuntimeShape input_shape = GetTensorShape(input);
    RuntimeShape output_shape = GetTensorShape(output);
    RuntimeShape filter_shape = GetTensorShape(filter);
    TFLITE_DCHECK_EQ(input_shape.DimensionsCount(), 4);
    TFLITE_DCHECK_EQ(filter_shape.DimensionsCount(), 4);
    TFLITE_DCHECK_EQ(output_shape.DimensionsCount(), 4);

    const int batch_size = MatchingDim(input_shape, 0, output_shape, 0);
    const int output_depth = MatchingDim(output_shape, 3, filter_shape, 3);
    TFLITE_DCHECK_EQ(batch_size, 1); /* Only batch = 1 is supported */

    cmsis_nn_dims input_dims;
    input_dims.n = batch_size;
    input_dims.h = input_height;
    input_dims.w = input_width;
    input_dims.c = input_shape.Dims(3);

    cmsis_nn_dims filter_dims;
    filter_dims.n = 1;
    filter_dims.h = filter_height;
    filter_dims.w = filter_width;
    filter_dims.c = output_depth;

    cmsis_nn_dims output_dims;
    output_dims.n = batch_size;
    output_dims.h = output_height;
    output_dims.w = output_width;
    output_dims.c = output_depth;

    cmsis_nn_dw_conv_params dw_conv_params;
    dw_conv_params.padding.h = data->reference_op_data.padding.height;
    dw_conv_params.padding.w = data->reference_op_data.padding.width;
    dw_conv_params.dilation.h = params.dilation_height_factor;
    dw_conv_params.dilation.w = params.dilation_width_factor;

    // 計算標準緩衝區大小
    int32_t buf_size = arm_depthwise_conv_wrapper_s8_get_buffer_size(
      &dw_conv_params, &input_dims, &filter_dims, &output_dims);
#if defined(ENABLE_DEPTH_CONV_NN_ACC)
    // 如果輸入通道數等於輸出通道數（深度乘子為1），檢查是否可以使用優化版本
    if (input_dims.c == output_dims.c && params.depth_multiplier == 1) {
      // 為 arm_depthwise_separable_conv_HWC_q7_nonsquare 計算所需的緩衝區大小
      int buffer_size_A = 2 * input_dims.c * filter_dims.w * filter_dims.h * sizeof(q15_t);
      int buffer_size_B = input_dims.c * sizeof(q7_t); // 可能需要調整
      int opt_buf_size = buffer_size_A + buffer_size_B;
      
      // 確保緩衝區足夠大
      buf_size = std::max(buf_size, opt_buf_size);
    }
#endif
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

inline void PopulateDwConvParams(
    cmsis_nn_dw_conv_params* const dw_conv_params,
    cmsis_nn_per_channel_quant_params* const quant_params,
    cmsis_nn_dims* const input_dims, cmsis_nn_dims* const filter_dims,
    cmsis_nn_dims* const bias_dims, cmsis_nn_dims* const output_dims,
    const TfLiteDepthwiseConvParams& params, const OpData& data,
    const TfLiteEvalTensor* input, const TfLiteEvalTensor* filter,
    const TfLiteEvalTensor* bias, TfLiteEvalTensor* output) {
  dw_conv_params->dilation.h = params.dilation_height_factor;
  dw_conv_params->dilation.w = params.dilation_width_factor;

  dw_conv_params->input_offset = -data.reference_op_data.input_zero_point;
  dw_conv_params->output_offset = data.reference_op_data.output_zero_point;
  dw_conv_params->stride.h = params.stride_height;
  dw_conv_params->stride.w = params.stride_width;
  dw_conv_params->padding.h = data.reference_op_data.padding.height;
  dw_conv_params->padding.w = data.reference_op_data.padding.width;

  dw_conv_params->activation.min = data.reference_op_data.output_activation_min;
  dw_conv_params->activation.max = data.reference_op_data.output_activation_max;

  dw_conv_params->ch_mult = params.depth_multiplier;

  quant_params->multiplier =
      data.reference_op_data.per_channel_output_multiplier;
  quant_params->shift = data.reference_op_data.per_channel_output_shift;

  RuntimeShape filter_shape = tflite::micro::GetTensorShape(filter);
  RuntimeShape input_shape = tflite::micro::GetTensorShape(input);
  RuntimeShape output_shape = tflite::micro::GetTensorShape(output);
  RuntimeShape bias_shape = tflite::micro::GetTensorShape(bias);

  TFLITE_DCHECK_LE(dw_conv_params->activation.min,
                   dw_conv_params->activation.max);

  const int batch_size = MatchingDim(input_shape, 0, output_shape, 0);
  const int output_depth = MatchingDim(filter_shape, 3, output_shape, 3);

  if (tflite::micro::GetOptionalTensorData<int8_t>(bias)) {
    TFLITE_DCHECK_EQ(bias_shape.FlatSize(), output_depth);
  }

  input_dims->n = batch_size;
  input_dims->h = input_shape.Dims(1);
  input_dims->w = input_shape.Dims(2);
  input_dims->c = input_shape.Dims(3);

  filter_dims->n = filter_shape.Dims(0);
  filter_dims->h = filter_shape.Dims(1);
  filter_dims->w = filter_shape.Dims(2);
  filter_dims->c = output_depth;

  bias_dims->n = 1;
  bias_dims->h = 1;
  bias_dims->w = 1;
  bias_dims->c = output_depth;

  output_dims->n = batch_size;
  output_dims->h = output_shape.Dims(1);
  output_dims->w = output_shape.Dims(2);
  output_dims->c = output_depth;
}

void EvalQuantizedPerChannel(TfLiteContext* context, TfLiteNode* node,
                             const TfLiteDepthwiseConvParams& params,
                             const OpData& data, const TfLiteEvalTensor* input,
                             const TfLiteEvalTensor* filter,
                             const TfLiteEvalTensor* bias,
                             TfLiteEvalTensor* output) {
  cmsis_nn_dw_conv_params dw_conv_params;
  cmsis_nn_per_channel_quant_params quant_params;
  cmsis_nn_dims input_dims;
  cmsis_nn_dims filter_dims;
  cmsis_nn_dims bias_dims;
  cmsis_nn_dims output_dims;

  PopulateDwConvParams(&dw_conv_params, &quant_params, &input_dims,
                       &filter_dims, &bias_dims, &output_dims, params, data,
                       input, filter, bias, output);
#if defined(ENABLE_DEPTH_CONV_NN_ACC)
  // 檢查是否可以使用優化的深度可分離卷積
  // 當輸入通道數等於輸出通道數時，且深度乘子為1時，可以使用
  if (input_dims.c == output_dims.c && params.depth_multiplier == 1) {
    const q7_t *Im_in = tflite::micro::GetTensorData<int8_t>(input);
    const q7_t *wt = tflite::micro::GetTensorData<int8_t>(filter);
    const q7_t *bias_data = tflite::micro::GetOptionalTensorData<int8_t>(bias);
    q7_t *Im_out = tflite::micro::GetTensorData<int8_t>(output);
    
    // 準備緩衝區
    q15_t* bufferA = nullptr;
    q7_t* bufferB = nullptr;
    
    cmsis_nn_context ctx;
    ctx.buf = nullptr;
    ctx.size = 0;

    if (data.buffer_idx > -1) {
      ctx.buf = context->GetScratchBuffer(context, data.buffer_idx);
      // bufferA 用於中間計算
      bufferA = static_cast<q15_t*>(ctx.buf);
      // bufferB 可能在某些實現中需要
      bufferB = reinterpret_cast<q7_t*>(bufferA + (input_dims.c * filter_dims.w * filter_dims.h));
    }

    // 計算偏移參數 - 這些值需要根據 TFLite 的量化方案進行調整
    int32_t effective_bias_shift = 0;
    int32_t effective_out_shift = -quant_params.shift[0]; // 注意：正負號可能需要調整
    
    // 調用 CMSIS-NN 加速函數
    arm_status status = arm_depthwise_separable_conv_HWC_q7_nonsquare(
        Im_in,
        input_dims.w,
        input_dims.h,
        input_dims.c,
        wt,
        output_dims.c,
        filter_dims.w,
        filter_dims.h,
        dw_conv_params.padding.w,
        dw_conv_params.padding.h,
        dw_conv_params.stride.w,
        dw_conv_params.stride.h,
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
      // 注意：CMSIS-NN 卷積函數可能已經應用了一些激活，但我們確保最終輸出在正確的範圍內
      int num_elements = output_dims.n * output_dims.h * output_dims.w * output_dims.c;
      for (int i = 0; i < num_elements; i++) {
        Im_out[i] = std::max<int8_t>(dw_conv_params.activation.min, 
                  std::min<int8_t>(dw_conv_params.activation.max, Im_out[i]));
      }
      
      return; // 成功完成計算，退出函數
    }
  }

  // 如果無法使用優化函數或優化函數失敗，回退到標準實現
#endif
  cmsis_nn_context ctx;
  ctx.buf = nullptr;
  /* 'size' is unused */
  ctx.size = 0;

  if (data.buffer_idx > -1) {
    ctx.buf = context->GetScratchBuffer(context, data.buffer_idx);
  }

  TFLITE_DCHECK_EQ(
      arm_depthwise_conv_wrapper_s8(
          &ctx, &dw_conv_params, &quant_params, &input_dims,
          tflite::micro::GetTensorData<int8_t>(input), &filter_dims,
          tflite::micro::GetTensorData<int8_t>(filter), &bias_dims,
          tflite::micro::GetOptionalTensorData<int32_t>(bias), &output_dims,
          tflite::micro::GetTensorData<int8_t>(output)),
      ARM_CMSIS_NN_SUCCESS);
}

void EvalQuantizedPerChannel16x8(TfLiteContext* context, TfLiteNode* node,
                                 const TfLiteDepthwiseConvParams& params,
                                 const OpData& data,
                                 const TfLiteEvalTensor* input,
                                 const TfLiteEvalTensor* filter,
                                 const TfLiteEvalTensor* bias,
                                 TfLiteEvalTensor* output) {
  cmsis_nn_dw_conv_params dw_conv_params;
  cmsis_nn_per_channel_quant_params quant_params;
  cmsis_nn_dims input_dims;
  cmsis_nn_dims filter_dims;
  cmsis_nn_dims bias_dims;
  cmsis_nn_dims output_dims;

  PopulateDwConvParams(&dw_conv_params, &quant_params, &input_dims,
                       &filter_dims, &bias_dims, &output_dims, params, data,
                       input, filter, bias, output);

  cmsis_nn_context ctx;
  ctx.buf = nullptr;
  /* 'size' is unused */
  ctx.size = 0;

  TFLITE_DCHECK_EQ(
      arm_depthwise_conv_s16(
          &ctx, &dw_conv_params, &quant_params, &input_dims,
          tflite::micro::GetTensorData<int16_t>(input), &filter_dims,
          tflite::micro::GetTensorData<int8_t>(filter), &bias_dims,
          tflite::micro::GetOptionalTensorData<int64_t>(bias), &output_dims,
          tflite::micro::GetTensorData<int16_t>(output)),
      ARM_CMSIS_NN_SUCCESS);
}

TfLiteStatus Eval(TfLiteContext* context, TfLiteNode* node) {
  TFLITE_DCHECK(node->user_data != nullptr);
  TFLITE_DCHECK(node->builtin_data != nullptr);

  const auto& params =
      *(reinterpret_cast<TfLiteDepthwiseConvParams*>(node->builtin_data));
  const OpData& data = *(static_cast<OpData*>(node->user_data));

  TfLiteEvalTensor* output =
      tflite::micro::GetEvalOutput(context, node, kDepthwiseConvOutputTensor);
  const TfLiteEvalTensor* input =
      tflite::micro::GetEvalInput(context, node, kDepthwiseConvInputTensor);
  const TfLiteEvalTensor* filter =
      tflite::micro::GetEvalInput(context, node, kDepthwiseConvWeightsTensor);
  const TfLiteEvalTensor* bias =
      (NumInputs(node) == 3)
          ? tflite::micro::GetEvalInput(context, node, kDepthwiseConvBiasTensor)
          : nullptr;

  TfLiteEvalTensor filter_int8 = tflite::micro::MakeUnpackedInt4Tensor(
      context, data.reference_op_data.filter_buffer_index, filter);

  switch (input->type) {  // Already know in/out types are same.
    case kTfLiteFloat32: {
      tflite::reference_ops::DepthwiseConv(
          DepthwiseConvParamsFloat(params, data.reference_op_data),
          tflite::micro::GetTensorShape(input),
          tflite::micro::GetTensorData<float>(input),
          tflite::micro::GetTensorShape(filter),
          tflite::micro::GetTensorData<float>(filter),
          tflite::micro::GetTensorShape(bias),
          tflite::micro::GetOptionalTensorData<float>(bias),
          tflite::micro::GetTensorShape(output),
          tflite::micro::GetTensorData<float>(output));
      break;
    }
    case kTfLiteInt8:
      switch (filter_int8.type) {
        case kTfLiteInt8: {
          EvalQuantizedPerChannel(context, node, params, data, input,
                                  &filter_int8, bias, output);
          break;
        }
        default: {
          MicroPrintf("Filter type %s (%d) not supported.",
                      TfLiteTypeGetName(filter->type), filter->type);
          return kTfLiteError;
        }
      }
      break;
    case kTfLiteInt16:
      EvalQuantizedPerChannel16x8(context, node, params, data, input, filter,
                                  bias, output);
      break;
    default:
      MicroPrintf("Type %s (%d) not supported.", TfLiteTypeGetName(input->type),
                  input->type);
      return kTfLiteError;
  }
  return kTfLiteOk;
}

TfLiteStatus EvalInt8(TfLiteContext* context, TfLiteNode* node) {
  TFLITE_DCHECK(node->user_data != nullptr);
  TFLITE_DCHECK(node->builtin_data != nullptr);

  const auto& params =
      *(reinterpret_cast<TfLiteDepthwiseConvParams*>(node->builtin_data));
  const OpData& data = *(static_cast<OpData*>(node->user_data));

  TfLiteEvalTensor* output =
      tflite::micro::GetEvalOutput(context, node, kDepthwiseConvOutputTensor);
  const TfLiteEvalTensor* input =
      tflite::micro::GetEvalInput(context, node, kDepthwiseConvInputTensor);
  const TfLiteEvalTensor* filter =
      tflite::micro::GetEvalInput(context, node, kDepthwiseConvWeightsTensor);
  const TfLiteEvalTensor* bias =
      (NumInputs(node) == 3)
          ? tflite::micro::GetEvalInput(context, node, kDepthwiseConvBiasTensor)
          : nullptr;

  TfLiteEvalTensor filter_int8 = tflite::micro::MakeUnpackedInt4Tensor(
      context, data.reference_op_data.filter_buffer_index, filter);

  EvalQuantizedPerChannel(context, node, params, data, input, &filter_int8,
                          bias, output);
  return kTfLiteOk;
}

TfLiteStatus EvalInt16x8(TfLiteContext* context, TfLiteNode* node) {
  TFLITE_DCHECK(node->user_data != nullptr);
  TFLITE_DCHECK(node->builtin_data != nullptr);

  const auto& params =
      *(reinterpret_cast<TfLiteDepthwiseConvParams*>(node->builtin_data));
  const OpData& data = *(static_cast<OpData*>(node->user_data));

  TfLiteEvalTensor* output =
      tflite::micro::GetEvalOutput(context, node, kDepthwiseConvOutputTensor);
  const TfLiteEvalTensor* input =
      tflite::micro::GetEvalInput(context, node, kDepthwiseConvInputTensor);
  const TfLiteEvalTensor* filter =
      tflite::micro::GetEvalInput(context, node, kDepthwiseConvWeightsTensor);
  const TfLiteEvalTensor* bias =
      (NumInputs(node) == 3)
          ? tflite::micro::GetEvalInput(context, node, kDepthwiseConvBiasTensor)
          : nullptr;

  EvalQuantizedPerChannel16x8(context, node, params, data, input, filter, bias,
                              output);
  return kTfLiteOk;
}

}  // namespace

TfLiteRegistration Register_DEPTHWISE_CONV_2D() {
  return tflite::micro::RegisterOp(Init, Prepare, Eval);
}

TfLiteRegistration Register_DEPTHWISE_CONV_2D_INT8() {
  return tflite::micro::RegisterOp(Init, Prepare, EvalInt8);
}

TfLiteRegistration Register_DEPTHWISE_CONV_2D_INT16() {
  return tflite::micro::RegisterOp(Init, Prepare, EvalInt16x8);
}

}  // namespace tflite
