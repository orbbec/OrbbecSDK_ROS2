/**
 * @file Version.h
 * @brief Functions for retrieving the SDK version number information.
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the SDK version number.
 *
 * @return int The SDK version number.
 */
int ob_get_version();

/**
 * @brief Get the SDK major version number.
 *
 * @return int The SDK major version number.
 */
int ob_get_major_version();

/**
 * @brief Get the SDK minor version number.
 *
 * @return int The SDK minor version number.
 */
int ob_get_minor_version();

/**
 * @brief Get the SDK patch version number.
 *
 * @return int The SDK patch version number.
 */
int ob_get_patch_version();

/**
 * @brief Get the SDK stage version.
 * @attention The returned char* does not need to be freed.
 *
 * @return const char* The SDK stage version.
 */
const char *ob_get_stage_version();

#ifdef __cplusplus
}
#endif