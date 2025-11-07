// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file Error.h
 * @brief Functions for handling errors, mainly used for obtaining error messages.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Create a new error object.
 *
 * @param[in] status The error status.
 * @param[in] message The error message.
 * @param[in] function The name of the API function that caused the error.
 * @param[in] args The error parameters.
 * @param[in] exception_type The type of exception that caused the error.
 *
 * @return ob_error* The new error object.
 */
OB_EXPORT ob_error *ob_create_error(ob_status status, const char *message, const char *function, const char *args, ob_exception_type exception_type);

/**
 * @brief Get the error status.
 *
 * @param[in] error The error object.
 * @return The error status.
 */
OB_EXPORT ob_status ob_error_get_status(const ob_error *error);

/**
 * @brief Get the error message.
 *
 * @param[in] error The error object.
 * @return The error message.
 */
OB_EXPORT const char *ob_error_get_message(const ob_error *error);

/**
 * @brief Get the name of the API function that caused the error.
 *
 * @param[in] error The error object.
 * @return The name of the API function.
 */
OB_EXPORT const char *ob_error_get_function(const ob_error *error);

/**
 * @brief Get the error parameters.
 *
 * @param[in] error The error object.
 * @return The error parameters.
 */
OB_EXPORT const char *ob_error_get_args(const ob_error *error);

/**
 * @brief Get the type of exception that caused the error.
 *
 * @param[in] error The error object.
 * @return The type of exception.
 */
OB_EXPORT ob_exception_type ob_error_get_exception_type(const ob_error *error);

/**
 * @brief Delete the error object.
 *
 * @param[in] error The error object to delete, you should set the pointer to NULL after calling this function.
 */
OB_EXPORT void ob_delete_error(ob_error *error);

// The following interfaces are deprecated and are retained here for compatibility purposes.
#define ob_error_status ob_error_get_status
#define ob_error_message ob_error_get_message
#define ob_error_function ob_error_get_function
#define ob_error_args ob_error_get_args
#define ob_error_exception_type ob_error_get_exception_type

#ifdef __cplusplus
}
#endif
