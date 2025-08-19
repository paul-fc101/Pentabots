#pragma once

#include <Arduino.h>

namespace mtrn3100 {
    class MovingAverageFilter {
    public:
        MovingAverageFilter() : curr(0), prev1(0), prev2(0) {}

        uint16_t average(uint16_t current) {
            prev2 = prev1;
            prev1 = curr;
            curr = current;

            currAvg = (curr + prev1 + prev2) / 3;
            currTime = millis();

            return currAvg;
        }

        uint16_t getAverage() {
            return currAvg;
        }

    private:
        uint16_t curr, prev1, prev2, currAvg;
        float currTime;

    };
}