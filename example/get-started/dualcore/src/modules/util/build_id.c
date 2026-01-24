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

#include "util/build_id.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

bool build_id_contains_gnu_build_id(const ElfExternalNote *note) {
  const uint32_t NT_GNU_BUILD_ID = 3;
  const char *expected_name = "GNU";
  const uint32_t expected_name_length = strlen(expected_name) + 1;
  return note->type == NT_GNU_BUILD_ID &&
         note->name_length == expected_name_length &&
         note->data_length == BUILD_ID_EXPECTED_LEN &&
         (strncmp((const char *) note->data, expected_name, expected_name_length) == 0);
}
