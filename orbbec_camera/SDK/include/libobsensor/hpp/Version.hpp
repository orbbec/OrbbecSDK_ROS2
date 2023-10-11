/**
 * @file Version.hpp
 * @brief Provides functions to retrieve version information of the SDK.
 */
#pragma once

namespace ob {
class OB_EXTENSION_API Version {
public:
    /**
     * @brief Get the major version number of the SDK.
     *
     * @return int The major version number of the SDK.
     */
    static int getMajor();

    /**
     * @brief Get the minor version number of the SDK.
     *
     * @return int The minor version number of the SDK.
     */
    static int getMinor();

    /**
     * @brief Get the patch version number of the SDK.
     *
     * @return int The patch version number of the SDK.
     */
    static int getPatch();

    /**
     * @brief Get the full version number of the SDK.
     *
     * @return int The full version number of the SDK.
     */
    static int getVersion();

    /**
     * @brief Get the stage version of the SDK.
     *
     * @return char* The stage version string of the SDK.
     */
    static char *getStageVersion();
};
}  // namespace ob
