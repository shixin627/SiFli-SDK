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
#include <stdbool.h>

#include "util/attributes.h"

//! The linker inserts the build id as an "elf external note" structure:
typedef struct PACKED {
  uint32_t name_length;
  uint32_t data_length;
  uint32_t type; // NT_GNU_BUILD_ID = 3
  uint8_t data[]; // concatenated name ('GNU') + data (build id)
} ElfExternalNote;

// the build id is a unique identification for the built files. The default algo uses SHA1
// to produce a 160 bit (20 byte sequence)
#define BUILD_ID_EXPECTED_LEN    (20)

#define BUILD_ID_NAME_EXPECTED_LEN  (4)

#define BUILD_ID_TOTAL_EXPECTED_LEN (sizeof(ElfExternalNote) + \
                                     BUILD_ID_NAME_EXPECTED_LEN + BUILD_ID_EXPECTED_LEN)

bool build_id_contains_gnu_build_id(const ElfExternalNote *note);
