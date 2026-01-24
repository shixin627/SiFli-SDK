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

#include <stdint.h>
#include <stddef.h>

//! Standard sort comparator function
typedef int (*SortComparator)(const void *, const void *);

//! Bubble sorts an array
//! @param[in] array The array that should be sorted
//! @param[in] num_elem Number of elements in the array
//! @param[in] elem_size Size of each element in the array
//! @param[in] comp SortComparator comparator function
void sort_bubble(void *array, size_t num_elem, size_t elem_size, SortComparator comp);
