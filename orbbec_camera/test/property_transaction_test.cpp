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

#include "orbbec_camera/property_transaction.h"

#include <stdexcept>
#include <string>

namespace {

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("property transaction test failed");
  }
}

}  // namespace

int main() {
  using orbbec_camera::setPropertyWithVerifiedRollback;

  int state = 1;
  const auto getter = [&state]() { return state; };
  const auto setter = [&state](int value) { state = value; };
  require(setPropertyWithVerifiedRollback(5, getter, setter) == 5);
  require(state == 5);

  state = 1;
  const auto reject_requested = [&state](int value) { state = value == 5 ? 2 : value; };
  try {
    setPropertyWithVerifiedRollback(5, getter, reject_requested);
    require(false);
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find("readback") != std::string::npos);
  }
  require(state == 1);

  state = 1;
  const auto reject_all = [&state](int) { state = 2; };
  try {
    setPropertyWithVerifiedRollback(5, getter, reject_all);
    require(false);
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find("rollback failed") != std::string::npos);
  }
  require(state == 2);

  return 0;
}
