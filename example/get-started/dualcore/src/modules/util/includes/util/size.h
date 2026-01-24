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

//! Calculate the length of an array, based on the size of the element type.
//! @param array The array to be evaluated.
//! @return The length of the array.
#define ARRAY_LENGTH(array) (sizeof((array))/sizeof((array)[0]))

//! Calculate the length of a literal array based on the size of the given type
//! This is usable in contexts that require compile time constants
//! @param type Type of the elements
//! @param array Literal definition of the array
//! @return Length of the array in bytes
#define STATIC_ARRAY_LENGTH(type, array) (sizeof((type[]) array) / sizeof(type))

#define MEMBER_SIZE(type, member) sizeof(((type *)0)->member)
