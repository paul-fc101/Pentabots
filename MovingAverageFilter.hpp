#pragma once

#include <Arduino.h>

namespace mtrn3100 {
    class MovingAverageFilter {
    public:
        MovingAverageFilter() : curr(0), prev1(0), prev2(0), prev3(0), prev4(0) {}

        uint16_t average(uint16_t current) {           
            prev4 = prev3;
            prev3 = prev2;
            prev2 = prev1;
            prev1 = curr;
            curr = current;

            uint16_t average = (curr + prev1 + prev2 + prev3 + prev4) / 5;

            return average;
        }

    private:
        uint16_t curr, prev1, prev2, prev3, prev4;
    };
}