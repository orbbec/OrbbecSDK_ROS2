/**
 * \if English
 * @file Error.h
 * @brief Error handling related functions, mainly used to obtain error messages
 * \else
 * @file Error.h
 * @brief 错误处理相关函数，主要用于获取错误信息
 * \endif
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * \if English
 * @brief Get error status
 *
 * @param[in] error  Error object
 * @return ob_status returns error status
 * \else
 * @brief 获取错误状态
 *
 * @param[in] error 错误对象
 * @return ob_status 返回错误状态
 * \endif
 */
ob_status ob_error_status(ob_error *error);
/**
 * \if English
 * @brief Get error information
 *
 * @param[in] error  Error object
 * @return const char* returns an error message
 * \else
 * @brief 获取错误信息
 *
 * @param[in] error 错误对象
 * @return const char* 返回错误信息
 * \endif
 */
const char *ob_error_message(ob_error *error);
/**
 * \if English
 * @brief Get error API function name
 *
 * @param[in] error Error object
 * @return const char* returns error function
 * \else
 * @brief 获取出现错误的API函数名称
 *
 * @param[in] error 错误对象
 * @return const char* 返回错误函数
 * \endif
 */
const char *ob_error_function(ob_error *error);
/**
 * \if English
 * @brief Get error parameters
 *
 * @param[in] error  Error object
 * @return const char* returns the error parameter
 * \else
 * @brief 获取错误参数
 *
 * @param[in] error 错误对象
 * @return const char* 返回错误参数
 * \endif
 */
const char *ob_error_args(ob_error *error);
/**
 * \if English
 * @brief Get error exception type
 *
 * @param[in] error Error object
 * @return ob_exception_type returns the error exception type
 * \else
 * @brief 获取错误异常类型
 *
 * @param[in] error 错误对象
 * @return ob_exception_type 返回错误异常类型
 * \endif
 */
ob_exception_type ob_error_exception_type(ob_error *error);
/**
 * \if English
 * @brief Delete the error object
 *
 * @param[in] error The error object to delete
 * \else
 * @brief 删除错误对象
 *
 * @param[in] error 要删除的错误对象
 * \endif
 */
void ob_delete_error(ob_error *error);

#ifdef __cplusplus
}
#endif