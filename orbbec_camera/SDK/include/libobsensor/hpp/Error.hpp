/**
 * \if English
 * @file  Error.hpp
 * @brief Error describes the abnormal error within the SDK, and the detailed information of the exception can be realized through this class.
 * \else
 * @file  Error.hpp
 * @brief Error描述SDK内部的异常错误，通过该类可以实现异常的详细信息。
 * \endif
 */
#pragma once

#include "Types.hpp"
#include <memory>

struct ErrorImpl;

namespace ob {
class OB_EXTENSION_API Error {
private:
    std::unique_ptr<ErrorImpl> impl_;

public:
    Error(std::unique_ptr<ErrorImpl> impl) noexcept;

    Error(const Error &error) noexcept;

    ~Error() noexcept;
    /**
     * \if English
     * @brief Obtain detailed error logs of SDK internal exceptions.
     * \else
     * @brief 获取SDK内部异常的详细错误日志。
     * \endif
     */
    const char *getMessage() const noexcept;

    /**
     * \if English
     * @brief Get the exception type of the error, and determine which module is abnormal.
     * @return OBExceptionType
     * \else
     * @brief 获取该错误的异常类型，判断是具体哪个模块异常。
     * @return OBExceptionType
     */
    OBExceptionType getExceptionType() const noexcept;

    /**
     * \if English
     * @brief Get the name of the error function.
     * \else
     * @brief 获取SDK内部异常的错误接口函数名称。
     * \endif
     */
    const char *getName() const noexcept;

    /**
     * \if English
     * @brief Get the parameter passed in to the error interface.
     * \else
     * @brief 获取SDK内部异常的错误接口函数传入参数。
     * \endif
     */
    const char *getArgs() const noexcept;
};
}  // namespace ob
