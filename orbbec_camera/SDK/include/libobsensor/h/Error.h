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
 * @brief Get the error status.
 *
 * @param[in] error The error object.
 * @return The error status.
 */
ob_status ob_error_status(ob_error *error);

/**
 * @brief Get the error message.
 *
 * @param[in] error The error object.
 * @return The error message.
 */
const char *ob_error_message(ob_error *error);

/**
 * @brief Get the name of the API function that caused the error.
 *
 * @param[in] error The error object.
 * @return The name of the API function.
 */
const char *ob_error_function(ob_error *error);

/**
 * @brief Get the error parameters.
 *
 * @param[in] error The error object.
 * @return The error parameters.
 */
const char *ob_error_args(ob_error *error);

/**
 * @brief Get the type of exception that caused the error.
 *
 * @param[in] error The error object.
 * @return The type of exception.
 */
ob_exception_type ob_error_exception_type(ob_error *error);

/**
 * @brief Delete the error object.
 *
 * @param[in] error The error object to delete.
 */
void ob_delete_error(ob_error *error);

#ifdef __cplusplus
}
#endif