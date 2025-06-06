#pragma once
#include <Arduino.h>
#include "robotka.h"
using namespace lx16a;

template<typename BusType>
void s_s_init(BusType& bus, int id, int low = 0, int high = 240) {
    bus.setAutoStop(id, false);
    bus.limit(id, Angle::deg(low), Angle::deg(high));
    lx16a::SmartServoBus::AutoStopParams params;
    params.max_diff_centideg = 400;
    params.max_diff_readings = 2;
    bus.setAutoStopParams(params);
    printf("Servo %d inicializováno\n", id);
}

template<typename BusType>
void s_s_move(BusType& bus, int id, int angle, int speed = 200.0) {
    if (angle < 0 || angle > 240) {
        printf("Chyba: Úhel musí být v rozsahu 0–240 stupňů.");
        return;
    }
    bus.setAutoStop(id, false);
    bus.set(id, Angle::deg(angle), speed);
    printf("Servo %d move na %d stupňů rychlostí %d\n", id, angle, speed);
}

template<typename BusType>
void s_s_soft_move(BusType& bus, int id, int angle, int speed = 200.0) {
    if (angle < 0 || angle > 240) {
        Serial.println("Chyba: Úhel musí být v rozsahu 0–240 stupňů.");
        return;
    }
    bus.setAutoStop(id, true);
    bus.set(id, Angle::deg(angle), speed);
    printf("Servo %d soft_move na %d stupňů rychlostí %d\n", id, angle, speed);
}