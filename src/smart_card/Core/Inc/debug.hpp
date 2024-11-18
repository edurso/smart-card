#pragma once

#include <string>
#include <cstdio>
#include <cstdarg>

namespace card {

    /**
     * Display a string to the serial console via UART
     * @param input the input to be displayed
     */
    inline auto debug(const std::string& input) -> void {
    	printf("%s\n\r", input.c_str());
    }

    /**
     * Display a character array to the serial console via UART
     * @param input the input to be displayed
     */
    inline auto debug(const char* input) -> void {
        printf("%s\n\r", input);
    }

    /**
     * Display a format string to the serial console via UART
     * @param format format string to display
     * @param ... format string values
     */
    inline auto debugf(const char* format, ...) -> void {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }

}
