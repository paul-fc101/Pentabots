#pragma once

#include <Arduino.h>

namespace mtrn3100 {

// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin 1 is attached to the interupt on the arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derrive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates. 
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4) : encoder1_int(enc1), encoder1_dir(enc2), encoder2_int(enc3), encoder2_dir(enc3) {
        instance = this;  // Store the instance pointer
        pinMode(encoder1_int, INPUT_PULLUP);
        pinMode(encoder1_dir, INPUT_PULLUP);
        pinMode(encoder2_int, INPUT_PULLUP);
        pinMode(encoder2_dir, INPUT_PULLUP);

        // TODO: attach the interrupt on pin one such that it calls the readEncoderISR function on a rising edge
        attachInterrupt(digitalPinToInterrupt(encoder1_int), readLeftEncoderISR, RISING);
        attachInterrupt(digitalPinToInterrupt(encoder2_int), readRightEncoderISR, RISING);
    }


    // Encoder function used to update the encoder
    void readLeftEncoder() {
        noInterrupts();
        direction = digitalRead(encoder1_dir) ? 1 : -1;
        l_count += direction;
        interrupts();
    }

    void readRightEncoder() {
        noInterrupts();
        direction = digitalRead(encoder2_dir) ? 1 : -1;
        r_count += direction;
        interrupts();
    }

    // Helper function which to convert encouder count to radians
    float getLeftRotation() {
        return (static_cast<float>(l_count) / counts_per_revolution ) * 2* PI;
    }

    float getRightRotation() {
        return (static_cast<float>(r_count) / counts_per_revolution ) * 2* PI;
    }

private:
    static void readLeftEncoderISR() {
        if (instance != nullptr) {
            instance->readLeftEncoder();
        }
    }
    static void readRightEncoderISR() {
        if (instance != nullptr) {
            instance->readRightEncoder();
        }
    }

public:
    const uint8_t encoder1_int, encoder1_dir, encoder2_int, encoder2_dir;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 729; //TODO: Identify how many encoder counts are in one rotation
    volatile long l_count = 0;
    volatile long r_count = 0;
    uint32_t prev_time;
    bool read = false;

private:
    static Encoder* instance;
};

Encoder* Encoder::instance = nullptr;

}  // namespace mtrn3100
