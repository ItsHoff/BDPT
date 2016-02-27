#include "util.hpp"
#include <iostream>


Statusbar::Statusbar(const std::string& descr, size_t max, float dispInterval)
        : max(max), imax(1.0f / max), interval(dispInterval), last(0) {
    std::cout << descr << " ... (" << max << ")" << std::endl;
}


void Statusbar::update(size_t val) {
    if ((val - last) * imax >= interval || val == max) {
        std::cout << (100.0f * val * imax) << " %\t\t\t\r" << std::flush;
        if (val == max)
            std::cout << std::endl;
        last = val;
    }
}
