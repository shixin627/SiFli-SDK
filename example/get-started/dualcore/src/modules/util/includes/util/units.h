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

 #pragma once

 #define MHZ_TO_HZ(hz) (((uint32_t)(hz)) * 1000000)
 
 #define KiBYTES(k) ((k) * 1024) // Kibibytes to Bytes
 #define MiBYTES(m) ((m) * 1024 * 1024) // Mebibytes to Bytes
 #define EiBYTES(e) ((e) * 1024 * 1024 * 1024 * 1024 * 1024 * 1024) // Exbibytes to Bytes
 
 #define MM_PER_METER 1000
 #define METERS_PER_KM 1000
 #define METERS_PER_MILE 1609
 
 #define US_PER_MS (1000)
 #define US_PER_S (1000000)
 #define NS_PER_S (1000000000)
 #define PS_PER_NS (1000)
 #define PS_PER_US (1000000)
 #define PS_PER_S (1000000000000ULL)
 