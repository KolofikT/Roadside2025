#include <Arduino.h>
#include "robotka.h"

using namespace lx16a; // aby nebylo třeba to psát všude
// Funkce pro inicializaci serva
// id: ID serva (0–253)
// low, high: Dolní a horní limit úhlu serva v ° (výchozí 0 a 240)
void s_s_init(auto & bus, int id, int low = 0, int high = 240) {
    bus.setAutoStop(id, false);
    bus.limit(id, Angle::deg(low), Angle::deg(high));
    bus.setAutoStopParams(
        SmartServoBus::AutoStopParams{//nastaveni sily stisku
            .max_diff_centideg = 400,
            .max_diff_readings = 2,
        });
    printf("Servo %d inicializováno\n", id);
}

// Funkce pro rychlý a přímý pohyb serva bez regulace
void s_s_move(auto & bus, int id, int angle, int speed = 200.0) {
    if (angle < 0 || angle > 240) {
        printf("Chyba: Úhel musí být v rozsahu 0–240 stupňů.");
        return;
    }
    bus.setAutoStop(id, false);
    bus.set(id, Angle::deg(angle), speed);
    printf("Servo %d move na %d stupňů rychlostí %d\n", id, angle, speed);
}

// Funkce pro plynulý pohyb serva s ochranou proti zaseknutí
void s_s_soft_move(auto & bus, int id, int angle, int speed = 200.0) {
    if (angle < 0 || angle > 240) {
        Serial.println("Chyba: Úhel musí být v rozsahu 0–240 stupňů.");
        return;
    }
    bus.setAutoStop(id, true);
    bus.set(id, Angle::deg(angle), speed);
    printf("Servo %d soft_move na %d stupňů rychlostí %d\n", id, angle, speed);
}