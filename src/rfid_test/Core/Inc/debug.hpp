#pragma once

#include <string>
#include <cstdio>
#include <cstdarg>

namespace card {

    inline void debug(const std::string& input) {
    	printf("%s\n\r", input.c_str());
    }

    inline void debug(const char* input) {
        printf("%s\n\r", input);
    }

    inline void debugf(const char* format, ...) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }

}
