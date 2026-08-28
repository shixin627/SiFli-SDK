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
#include <stdint.h>
#include "gesture_predictor.h"
#include "constants.h"
#include "rtdbg.h"
namespace
{
	// State for the averaging algorithm we're using.
	float prediction_history[kGestureCount][kPredictionHistoryLength] = {};
	int prediction_history_index = 0;
	int prediction_suppression_count = 0;
} // namespace

// Return the result of the last prediction
// 0: unknown, 1: ring("O"), 2: slope("angle"), 3: unknown
// int PredictGesture(float* output) {
//    // Record the latest predictions in our rolling history buffer.
//    for (int i = 0; i < kGestureCount; ++i) {
//        prediction_history[i][prediction_history_index] = output[i];
//    }
//    // Figure out which slot to put the next predictions into.
//    ++prediction_history_index;
//    if (prediction_history_index >= kPredictionHistoryLength) {
//        prediction_history_index = 0;
//    }

//    // Average the last n predictions for each gesture, and find which has the
//    // highest score.
//    int max_predict_index = -1;
//    float max_predict_score = 0.0f;
//    for (int i = 0; i < kGestureCount; i++) {
//        float prediction_sum = 0.0f;
//        for (int j = 0; j < kPredictionHistoryLength; ++j) {
//            prediction_sum += prediction_history[i][j];
//        }
//        const float prediction_average = prediction_sum / kPredictionHistoryLength;
//        if ((max_predict_index == -1) || (prediction_average > max_predict_score)) {
//            max_predict_index = i;
//            max_predict_score = prediction_average;
//        }
//    }

//    // If there's been a recent prediction, don't trigger a new one too soon.
//    if (prediction_suppression_count > 0) {
//        --prediction_suppression_count;
//    }
//    // If we're predicting no gesture, or the average score is too low, or there's
//    // been a gesture recognised too recently, return no gesture.
//    if ((max_predict_index == kNoGesture) ||
//        (max_predict_score < kDetectionThreshold) ||
//        (prediction_suppression_count > 0)) {
//        return kNoGesture;
//    }
//    else {
//        // Reset the suppression counter so we don't come up with another prediction
//        // too soon.
//        prediction_suppression_count = kPredictionSuppressionDuration;
//				DBG_DIRECT("Got max predict index = %d, score = %f", max_predict_index, output[max_predict_index]);
//        return max_predict_index;
//    }
//}

/* Tap 的信心門檻(2026-08-28)。可用 MSH `gtap thr <0..100>` 即時調。

   為什麼一定要有:三類 softmax 的 argmax 最低只要 0.34 就會贏,等於「不確定」
   這個選項根本不存在 —— 每一個切出來的視窗都會被指派成 tap/release 其中之一。
   舊 arming 每分鐘切 20 個視窗、其中大多數不是手勢,所以誤報是結構性的。

   門檻值來自 mouse3 `data3/mouse` 的留一人交叉驗證(8 人 153 分鐘、FSR 真值),
   掃 P(tap) 得到整條 recall–誤報曲線。**5 個訓練種子重跑**的結果(出貨只放一顆
   模型,所以要看的是多種子的平均與散布,不是最好的那一次):

     誤報預算   recall(2445 個按壓)
     ≤0.5/分    0.390 ± 0.086
     ≤1.0/分    0.505 ± 0.050
     ≤2.0/分    0.587 ± 0.025
     ≤3.66/分   0.633 ± 0.014   ← 3.66 = 現行系統自己的誤報率(recall 0.367)

   **門檻不是照上表挑的**。上面那份資料的負樣本全是「正在操作滑鼠的手」,而真機
   最在意的誤報情境是「手在動但根本沒要按」。`data3/free`(自由活動、完全沒按,
   只有 4.5 分鐘)量出來,free 的誤報對門檻遠比 mouse 敏感:

     門檻   mouse recall   mouse 誤報/分   free 誤報/分
     0.55      0.585           1.57           3.48   ← 比現行的 1.78 還糟
     0.65      0.555           1.14           2.59
     0.70      0.537           0.94           2.07
     0.75      0.514           0.77           1.70   ← 三個軸都不輸現行
     現行      0.367           3.66           1.78

   所以出貨取 **0.75**:recall 0.367→0.514(+40%)、mouse 誤報 3.66→0.77(−79%)、
   free 誤報 1.78→1.70(不退步)。這是保守取法 —— free 只有 4.5 分鐘、8 vs 13 次
   在統計上分不出來,但既然分不出來就不該拿它換 recall。等有幾十分鐘的日常負樣本
   (走路/打字/通勤)再重挑,`gtap thr` 不必重編。
   低於門檻一律回 kUnknownGesture(2),不是回 0 —— 0 是 release 類別。 */
static float tap_confidence_threshold = 0.75f;
extern "C" void gesture_set_tap_confidence(float thr) { tap_confidence_threshold = thr; }
extern "C" float gesture_get_tap_confidence(void) { return tap_confidence_threshold; }

// online prediction
int PredictGesture(float *output) // 1*3 [probability of 3 gestures]
{
	float max_gesture_score = 0;
	int max_gesture_score_index = 0;
	for (uint8_t i = 0; i < kGestureCount; i++)
	{
		if (output[i] > max_gesture_score)
		{
			max_gesture_score = output[i];
			max_gesture_score_index = i;
		}
	}
	/* 只擋 tap:release 那條路徑走的是解鎖/開麥克風,誤觸的代價低很多,而且
	   它的視窗量本來就少(RELEASE 抽取器只在遊戲模式/motion lock 時才跑)。 */
	if (max_gesture_score_index == kTapGesture &&
	    max_gesture_score < tap_confidence_threshold)
	{
		return kUnknownGesture;
	}
	return max_gesture_score_index;
}

int PredictQuatizedGesture(int16_t *output)
{
	float max_gesture_score = 0;
	int max_gesture_score_index = 0;
	for (uint8_t i = 0; i < kGestureCount; i++)
	{
		if (output[i] > max_gesture_score)
		{
			max_gesture_score = output[i];
			max_gesture_score_index = i;
		}
	}
	return max_gesture_score_index;
}