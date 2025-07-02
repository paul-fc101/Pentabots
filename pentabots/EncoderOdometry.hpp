#pragma once

#include <Arduino.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0) {}

    //TODO: COMPLETE THIS FUNCTION
    void update(float leftValue,float rightValue) {
        float tL = leftValue - lastLPos; // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 
        float tR = rightValue - lastRPos; // MAKE SURE THE ENCODER COUNT UP CORRECTLY / ARE NOT THE WRONG DIRECTION 
         

        float s = R*tL/2 + R*tR/2;
        
        float th = -R*tL/(B*2) + R*tR/(2*B);

        x += s*cos(th);
        y += s*sin(th);
        h += th;
        lastLPos = leftValue;
        lastRPos = rightValue;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

private:
    float x, y, h;
    const float R, B;
    float lastLPos, lastRPos;
};

}