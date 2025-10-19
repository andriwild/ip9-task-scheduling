#pragma once
#include <iostream>

#include <iostream>
#include <string>

namespace Log {
    inline void d(const std::string& msg) {
        std::cout << "\033[38;2;150;200;220m[DEBUG]\033[0m " << msg << std::endl;
    }

    inline void i(const std::string& msg) {
        std::cout << "\033[38;2;120;160;220m[INFO]\033[0m " << msg << std::endl;
    }

    inline void w(const std::string& msg) {
        std::cout << "\033[38;2;240;200;120m[WARN] " << msg << "\033[0m" << std::endl;
    }

    inline void e(const std::string& msg) {
        std::cerr << "\033[38;2;240;130;130m[ERROR] " << msg << "\033[0m" << std::endl;
    }

    inline void s(const std::string& msg) {
        std::cout << "\033[38;2;150;220;150m[SUCCESS]\033[0m " << msg << std::endl;
    }
}