#pragma once

#include <string>
#include <stdio.h>

namespace card {

    void display(const std::string& input) {
    	printf("%s\n\r", input.c_str());
    }

    void display(const char* input) {
        printf("%s\n\r", input);
    }

}
