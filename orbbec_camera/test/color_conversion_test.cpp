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

#include <array>
#include <cstdint>
#include <stdexcept>

#include "orbbec_camera/color_conversion.h"

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("color conversion contract failed");
  }
}

}  // namespace

int main() {
  {
    const std::array<uint8_t, 4> yuyv{8, 128, 16, 128};
    std::array<uint8_t, 6> rgb{};
    require(
        orbbec_camera::yuyvFullRangeToRgb(yuyv.data(), yuyv.size(), rgb.data(), rgb.size(), 2, 1));
    require((rgb == std::array<uint8_t, 6>{8, 8, 8, 16, 16, 16}));
  }

  {
    const std::array<uint8_t, 4> yuyv{0, 128, 255, 128};
    std::array<uint8_t, 6> rgb{};
    require(
        orbbec_camera::yuyvFullRangeToRgb(yuyv.data(), yuyv.size(), rgb.data(), rgb.size(), 2, 1));
    require((rgb == std::array<uint8_t, 6>{0, 0, 0, 255, 255, 255}));
  }

  {
    const std::array<uint8_t, 4> yuyv{100, 255, 100, 0};
    std::array<uint8_t, 6> rgb{};
    require(
        orbbec_camera::yuyvFullRangeToRgb(yuyv.data(), yuyv.size(), rgb.data(), rgb.size(), 2, 1));
    require(rgb[0] == 0);
    require(rgb[1] > 140);
    require(rgb[2] == 255);
  }

  {
    std::array<uint8_t, 4> yuyv{};
    std::array<uint8_t, 6> rgb{};
    require(
        !orbbec_camera::yuyvFullRangeToRgb(yuyv.data(), yuyv.size(), rgb.data(), rgb.size(), 1, 1));
    require(!orbbec_camera::yuyvFullRangeToRgb(yuyv.data(), 3, rgb.data(), rgb.size(), 2, 1));
    require(!orbbec_camera::yuyvFullRangeToRgb(yuyv.data(), yuyv.size(), rgb.data(), 5, 2, 1));
  }
  return 0;
}
