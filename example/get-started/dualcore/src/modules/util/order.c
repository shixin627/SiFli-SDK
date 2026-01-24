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

#include "util/order.h"

int uint32_comparator(void *a, void *b) {
  uint32_t A = *(uint32_t *)a;
  uint32_t B = *(uint32_t *)b;

  if (B > A) {
    return 1;
  } else if (B < A) {
    return -1;
  } else {
    return 0;
  }
}
