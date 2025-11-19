// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

/**
 * @file  Error.hpp
 * @brief This file defines the Error class, which describes abnormal errors within the SDK.
 *        Detailed information about the exception can be obtained through this class.
 */
#pragma once

#include "Types.hpp"
#include "libobsensor/h/Error.h"
#include <memory>

namespace ob {
class Error : public std::exception {
private:
    ob_error *impl_;

    /**
     * @brief Construct a new Error object
     *
     * @attention This constructor should not be called directly, use the handle() function instead.
     *
     * @param[in] error The ob_error object
     */
    explicit Error(ob_error *error) : impl_(error) {};

    Error& operator=(const Error&) = default;

public:
    /**
     * @brief A static function to handle the ob_error and throw an exception if needed.
     *
     * @param[in,out] error The ob_error pointer to be handled.
     * @param[in] throw_exception A boolean value to indicate whether to throw an exception or not, the default value is true.
     */
    static void handle(ob_error **error, bool throw_exception = true) {
        if(!error || !*error) {  // no error
            return;
        }

        if(throw_exception) {
            throw Error(*error);
        }
        else {
            ob_delete_error(*error);
            *error = nullptr;
        }
    }

    /**
     * @brief Destroy the Error object
     */
    ~Error() override {
        if(impl_) {
            ob_delete_error(impl_);
            impl_ = nullptr;
        }
    }

    /**
     * @brief Returns the error message of the exception.
     *
     * @return const char* The error message.
     */
    const char *what() const noexcept override {
        return impl_->message;
    }

    /**
     * @brief Returns the exception type of the exception.
     * @brief Read the comments of the OBExceptionType enum in the libobsensor/h/ObTypes.h file for more information.
     *
     * @return OBExceptionType The exception type.
     */
    OBExceptionType getExceptionType() const noexcept {
        return impl_->exception_type;
    }

    /**
     * @brief Returns the name of the function where the exception occurred.
     *
     * @return const char* The function name.
     */
    const char *getFunction() const noexcept {
        return impl_->function;
    }

    /**
     * @brief Returns the arguments of the function where the exception occurred.
     *
     * @return const char*  The arguments.
     */
    const char *getArgs() const noexcept {
        return impl_->args;
    }

    /**
     * @brief Returns the error message of the exception.
     * @brief It is recommended to use the what() function instead.
     *
     * @return const char* The error message.
     */
    const char *getMessage() const noexcept {
        return impl_->message;
    }

public:
    // The following interfaces are deprecated and are retained here for compatibility purposes.
    const char *getName() const noexcept {
        return impl_->function;
    }
};
}  // namespace ob

