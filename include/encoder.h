#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(int pinA, int pinB);
    void update();
    int32_t getCount();
    float getWheelVelocity(float wheel_radius, float dt);

private:
    int pinA, pinB;
    volatile int32_t count;
    int lastState;
};

#endif // ENCODER_H
