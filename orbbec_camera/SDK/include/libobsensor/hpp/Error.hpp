/**
 * @file  Error.hpp
 * @brief This file defines the Error class, which describes abnormal errors within the SDK.
 *        Detailed information about the exception can be obtained through this class.
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
     * @brief Get the detailed error logs of SDK internal exceptions.
     * @return A C-style string containing the error message.
     */
    const char *getMessage() const noexcept;

    /**
     * @brief Get the exception type of the error, which can be used to determine which module is abnormal.
     * @return The OBExceptionType enum value.
     */
    OBExceptionType getExceptionType() const noexcept;

    /**
     * @brief Get the name of the error function.
     * @return A C-style string containing the name of the error function.
     */
    const char *getName() const noexcept;

    /**
     * @brief Get the parameter passed to the error interface.
     * @return A C-style string containing the error interface parameter.
     */
    const char *getArgs() const noexcept;
};
}  // namespace ob
