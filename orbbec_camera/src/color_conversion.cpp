/*******************************************************************************
 * Copyright (c) 2026 Orbbec 3D Technology, Inc
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
 *******************************************************************************/

#include "orbbec_camera/color_conversion.h"

#include <algorithm>

namespace orbbec_camera {
namespace {

uint8_t clampToByte(int value) { return static_cast<uint8_t>(std::clamp(value, 0, 255)); }

void yuvToRgb(uint8_t y, int u, int v, uint8_t* rgb) {
  rgb[0] = clampToByte(static_cast<int>(y) + 359 * v / 256);
  rgb[1] = clampToByte(static_cast<int>(y) - (88 * u + 183 * v) / 256);
  rgb[2] = clampToByte(static_cast<int>(y) + 454 * u / 256);
}

}  // namespace

bool yuyvFullRangeToRgb(const uint8_t* source, size_t source_size, uint8_t* destination,
                        size_t destination_size, uint32_t width, uint32_t height) {
  const size_t pixel_count = static_cast<size_t>(width) * height;
  if (source == nullptr || destination == nullptr || width == 0 || height == 0 || width % 2 != 0 ||
      source_size < pixel_count * 2 || destination_size < pixel_count * 3) {
    return false;
  }

  for (size_t source_offset = 0, destination_offset = 0; source_offset < pixel_count * 2;
       source_offset += 4, destination_offset += 6) {
    const int u = static_cast<int>(source[source_offset + 1]) - 128;
    const int v = static_cast<int>(source[source_offset + 3]) - 128;
    yuvToRgb(source[source_offset], u, v, destination + destination_offset);
    yuvToRgb(source[source_offset + 2], u, v, destination + destination_offset + 3);
  }
  return true;
}

}  // namespace orbbec_camera
