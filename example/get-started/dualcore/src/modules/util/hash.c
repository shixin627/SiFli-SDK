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

#include "util/hash.h"

#include <stdint.h>

// Based on DJB2 Hash
uint32_t hash(const uint8_t *bytes, const uint32_t length) {
  uint32_t hash = 5381;

  if (length == 0) {
    return hash;
  }

  uint8_t c;
  const uint8_t *last_byte = bytes + length;
  while (bytes != last_byte) {
    c = *bytes;
    hash = ((hash << 5) + hash) + c;
    bytes++;
  }
  return hash;
}
