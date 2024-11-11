#pragma once

#include <string>
#include <stdio.h>

namespace card {
    inline void display(const std::string& input) {
    	printf("%s\n\r", input.c_str());
    }

    inline void display(const char* input) {
        printf("%s\n\r", input);
    }

    inline std::string getHexId(const std::uint8_t id[4]) {
        std::string result;
        for (int i = 0; i < 4; ++i) {
            const char high = (id[i] >> 4) + ((id[i] >> 4) < 10 ? '0' : 'A' - 10);
            const char low = (id[i] & 0x0F) + ((id[i] & 0x0F) < 10 ? '0' : 'A' - 10);
            result += high;
            result += low;
            if (i < 3) result += ":";
        }
        return result;
    }

}
