/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "hr_util.h"

#include "activity.h"

// ------------------------------------------------------------------------------------------------
HRZone hr_util_get_hr_zone(int bpm) {
  const int zone_thresholds[HRZone_Max] = {
    activity_prefs_heart_get_zone1_threshold(),
    activity_prefs_heart_get_zone2_threshold(),
    activity_prefs_heart_get_zone3_threshold(),
  };

  HRZone zone;
  for (zone = HRZone_Zone0; zone < HRZone_Max; zone++) {
    if (bpm < zone_thresholds[zone]) {
      break;
    }
  }
  return zone;
}

bool hr_util_is_elevated(int bpm) {
  return bpm >= activity_prefs_heart_get_elevated_hr();
}
