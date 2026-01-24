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

#include "util/iterator.h"

#include "util/assert.h"

#include <stdbool.h>

void iter_init(Iterator* iter, IteratorCallback next, IteratorCallback prev, IteratorState state) {
  *iter = (Iterator) {
    .next = next,
    .prev = prev,
    .state = state
  };
}

bool iter_next(Iterator* iter) {
  UTIL_ASSERT(iter->next);
  return iter->next(iter->state);
}

bool iter_prev(Iterator* iter) {
  UTIL_ASSERT(iter->prev);
  return iter->prev(iter->state);
}

