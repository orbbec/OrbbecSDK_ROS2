/**
 * \if English
 * @file Version.h
 * @brief Get the SDK version number information function
 * \else
 * @file Version.h
 * @brief 获取SDK版本号相关信息函数
 * \endif
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \if English
 * @brief Get the SDK version
 *
 * @return int  returns the SDK version
 * \else
 * @brief 获取SDK版本号
 *
 * @return int 返回SDK版本号
 * \endif
 */
int ob_get_version();

/**
 * \if English
 * @brief Get the SDK major version
 *
 * @return int returns the SDK major version
 * \else
 * @brief 获取SDK主版本号
 *
 * @return int 返回SDK主版本号
 * \endif
 */
int ob_get_major_version();
/**
 * \if English
 * @brief Get the SDK minor version
 *
 * @return int returns the SDK minor version
 * \else
 * @brief 获取SDK副版本号
 *
 * @return int 返回SDK副版本号
 * \endif
 */
int ob_get_minor_version();

/**
 * \if English
 * @brief Get the SDK patch version
 *
 * @return int returns the SDK patch version
 * \else
 * @brief 获取SDK修订版本号
 *
 * @return int 返回SDK修订版本号
 * \endif
 */
int ob_get_patch_version();

#ifdef __cplusplus
}
#endif