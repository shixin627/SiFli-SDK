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

//! Simple utility for enforcing consistent use of the iterator pattern
//! and facilitate unit testing.
#pragma once

#include <stdbool.h>

typedef void* IteratorState;

typedef bool (*IteratorCallback)(IteratorState state);

typedef struct {
  IteratorCallback next;
  IteratorCallback prev;
  IteratorState state;
} Iterator;

#define ITERATOR_EMPTY ((Iterator){ 0, 0, 0 })

void iter_init(Iterator* iter, IteratorCallback next, IteratorCallback prev, IteratorState state);

//! @return true if successfully moved to next node
bool iter_next(Iterator* iter);

//! @return true if successfully moved to previous node
bool iter_prev(Iterator* iter);

IteratorState iter_get_state(Iterator* iter);

