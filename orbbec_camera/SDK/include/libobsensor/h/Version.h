// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Version.h
 * @brief Functions for retrieving the SDK version number information.
 *
 */
#pragma once

#include "Export.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the SDK version number.
 *
 * @return int The SDK version number.
 */
OB_EXPORT int ob_get_version(void);

/**
 * @brief Get the SDK major version number.
 *
 * @return int The SDK major version number.
 */
OB_EXPORT int ob_get_major_version(void);

/**
 * @brief Get the SDK minor version number.
 *
 * @return int The SDK minor version number.
 */
OB_EXPORT int ob_get_minor_version(void);

/**
 * @brief Get the SDK patch version number.
 *
 * @return int The SDK patch version number.
 */
OB_EXPORT int ob_get_patch_version(void);

/**
 * @brief Get the SDK stage version.
 *
 * @attention The returned char* does not need to be freed.
 *
 * @return const char* The SDK stage version.
 */
OB_EXPORT const char *ob_get_stage_version(void);

#ifdef __cplusplus
}
#endif
