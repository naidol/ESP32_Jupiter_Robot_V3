#include "encoder.h"
#include "Arduino.h"

Encoder::Encoder(int pinA, int pinB) : pinA(pinA), pinB(pinB), count(0), lastState(LOW) {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    lastState = digitalRead(pinA);
}

void Encoder::update() {
    int currentState = digitalRead(pinA);
    if (currentState != lastState) {
        if (digitalRead(pinB) != currentState) {
            count++;
        } else {
            count--;
        }
    }
    lastState = currentState;
}

int32_t Encoder::getCount() {
    return count;
}

float Encoder::getWheelVelocity(float wheel_radius, float dt) {
    float distance_per_tick = 2 * PI * wheel_radius / 360.0; // Assuming 360 ticks per revolution
    return (count * distance_per_tick) / dt;
}
