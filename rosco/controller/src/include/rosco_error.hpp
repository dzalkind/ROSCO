// rosco_error.hpp — C++ exception types for ROSCO error handling.
//
// RoscoError replaces the Fortran-heritage aviFAIL = -1 / ErrMsg pattern.
// Thrown inside controller modules, caught at the DISCON boundary and
// converted to the aviFAIL/avcMSG channel that Bladed/OpenFAST expects.

#pragma once
#include <stdexcept>
#include <string>
#include <cstdio>
#include <cstdarg>

// Fatal controller error — caught at the DISCON boundary → aviFAIL = -1
class RoscoError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Throw a RoscoError with printf-style formatting and routine-name prefix.
// Usage: rosco_throw("CheckInputs", "LoggingLevel must be 0 - 3.");
//        rosco_throw("interp1d", "SIZE(xData) =%2d and SIZE(yData) =%2d are not the same", nx, ny);
inline void rosco_throw(const char* routine, const char* fmt, ...) {
    char buf[1024];
    int offset = std::snprintf(buf, sizeof(buf), "%s: ", routine);
    if (offset < 0) offset = 0;
    if (offset < (int)sizeof(buf)) {
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buf + offset, sizeof(buf) - offset, fmt, args);
        va_end(args);
    }
    throw RoscoError(buf);
}

// Non-fatal warning — prints to stderr. Used for restart I/O failures
// where the controller can continue without the data.
inline void rosco_warn(const char* routine, const char* fmt, ...) {
    char buf[1024];
    int offset = std::snprintf(buf, sizeof(buf), "ROSCO WARNING [%s]: ", routine);
    if (offset < 0) offset = 0;
    if (offset < (int)sizeof(buf)) {
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buf + offset, sizeof(buf) - offset, fmt, args);
        va_end(args);
    }
    std::fprintf(stderr, "%s\n", buf);
}
