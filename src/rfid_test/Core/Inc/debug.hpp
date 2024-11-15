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

    inline std::string trim(const std::string& str,
                            const std::string& whitespace = " \t")
    {
        const auto strBegin = str.find_first_not_of(whitespace);
        if (strBegin == std::string::npos)
            return ""; // no content

        const auto strEnd = str.find_last_not_of(whitespace);
        const auto strRange = strEnd - strBegin + 1;

        return str.substr(strBegin, strRange);
    }

    inline std::string reduce(const std::string& str,
                              const std::string& fill = " ",
                              const std::string& whitespace = " \t")
    {
        // trim first
        auto result = trim(str, whitespace);

        // replace sub ranges
        auto beginSpace = result.find_first_of(whitespace);
        while (beginSpace != std::string::npos)
        {
            const auto endSpace = result.find_first_not_of(whitespace, beginSpace);
            const auto range = endSpace - beginSpace;

            result.replace(beginSpace, range, fill);

            const auto newStart = beginSpace + fill.length();
            beginSpace = result.find_first_of(whitespace, newStart);
        }

        return result;
    }

}
