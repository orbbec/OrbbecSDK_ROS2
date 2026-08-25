/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
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

#pragma once

#include <exception>
#include <stdexcept>
#include <string>

namespace orbbec_camera {

template <typename T, typename Getter, typename Setter>
T setPropertyWithVerifiedRollback(const T &requested, Getter getter, Setter setter) {
  const T previous = getter();
  try {
    setter(requested);
    const T applied = getter();
    if (applied != requested) {
      throw std::runtime_error("property readback does not match requested value");
    }
    return applied;
  } catch (...) {
    const auto original_error = std::current_exception();
    try {
      setter(previous);
      if (getter() != previous) {
        throw std::runtime_error("rollback readback does not match previous value");
      }
    } catch (const std::exception &rollback_error) {
      throw std::runtime_error("property update failed and rollback failed: " +
                               std::string(rollback_error.what()));
    } catch (...) {
      throw std::runtime_error("property update failed and rollback failed with unknown error");
    }
    std::rethrow_exception(original_error);
  }
}

}  // namespace orbbec_camera
