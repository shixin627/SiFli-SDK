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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*HexdumpLineCallback)(int level, const char *src_filename, int src_line_number,
                                    const char *line_buffer);

//! Hexdumps data in xxd-style formatting, by repeatedly calling write_line_cb for each line.
//! @note The line_buffer that is passed does not end with any newline characters.
void hexdump(const char *src_filename, int src_line_number, int level,
             const uint8_t *data, size_t length, HexdumpLineCallback write_line_cb);
