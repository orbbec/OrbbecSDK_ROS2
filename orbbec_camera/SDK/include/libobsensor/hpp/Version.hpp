/**
 * \if English
 * @file Version.hpp
 * @brief Get the SDK version information
 * \else
 * @file Version.hpp
 * @brief 获取SDK版本号相关信息的类
 * \endif
 */
#pragma once

namespace ob {
class OB_EXTENSION_API Version {
public:
    /**
     * \if English
     * @brief Get the SDK major version number
     *
     * @return int returns the SDK major version number
     * \else
     * @brief 获取SDK主版本号
     *
     * @return int 返回SDK主版本号
     * \endif
     */
    static int getMajor();
    /**
     * \if English
     * @brief Get the SDK minor version number
     *
     * @return int returns the SDK minor version number
     * \else
     * @brief 获取SDK副版本号
     *
     * @return int 返回SDK副版本号
     * \endif
     */
    static int getMinor();
    /**
     * \if English
     * @brief Get the SDK revision number
     *
     * @return int returns the SDK revision number
     * \else
     * @brief 获取SDK修订版本号
     *
     * @return int 返回SDK修订版本号
     * \endif
     */
    static int getPatch();

    /**
     * \if English
     * @brief Get the SDK version number
     *
     * @return int returns the SDK version number
     * \else
     * @brief 获取SDK版本号
     *
     * @return int 返回SDK版本号
     * \endif
     */
    static int getVersion();
};
}  // namespace ob
