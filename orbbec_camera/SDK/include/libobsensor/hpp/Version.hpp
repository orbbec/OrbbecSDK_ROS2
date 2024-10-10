// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Version.hpp
 * @brief Provides functions to retrieve version information of the SDK.
 */
#pragma once

#include "libobsensor/h/Version.h"

namespace ob {
class Version {
public:
    /**
     * @brief Get the full version number of the SDK.
     * @brief The full version number equals to: major * 10000 + minor * 100 + patch
     *
     * @return int The full version number of the SDK.
     */
    static int getVersion() {
        return ob_get_version();
    }
    /**
     * @brief Get the major version number of the SDK.
     *
     * @return int The major version number of the SDK.
     */
    static int getMajor() {
        return ob_get_major_version();
    }

    /**
     * @brief Get the minor version number of the SDK.
     *
     * @return int The minor version number of the SDK.
     */
    static int getMinor() {
        return ob_get_minor_version();
    }

    /**
     * @brief Get the patch version number of the SDK.
     *
     * @return int The patch version number of the SDK.
     */
    static int getPatch() {
        return ob_get_patch_version();
    }

    /**
     * @brief Get the stage version of the SDK.
     * @brief The stage version string is a vendor defined string for some special releases.
     *
     * @return char* The stage version string of the SDK.
     */
    static const char *getStageVersion() {
        return ob_get_stage_version();
    }
};
}  // namespace ob

