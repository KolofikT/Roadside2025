//RBCX - Robotka
#include <Arduino.h>
#include "robotka.h"
#include "smart_servo_command.h"
#include "motor_commands.h"
#include "RBCXMotor.h"
#include "RBCX.h"

#include "Movement.hpp"

#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>

//Pixy2 kamera
#include <Pixy2.h>
#include <SPI.h>

//Docks
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <algorithm> // pro std::min_element
#include <climits>   // pro INT_MAX

/***********************************************************************************************************************/

Movement move;

using namespace lx16a; // aby nebylo třeba to psát všude

/***********************************************************************************************************************/

    // Baterie:
    //    Row1: 7ks   420mm výška 60mm (100mm i s kroužkem)
    //    Row2: 7ks   420mm výška 60mm (100mm i s kroužkem)
    // 
    // Cesta:
    //    Šířka:  400mm
    //    Délka:  2000mm
    //
    // Dock:
    //    120 * 120 * 80mm
    //    120mm od zdi
    //    75mm mezi docky

    //      Rameno od zdi při startu        210mm              
    //
    //    1.  dock  800mm     od zdi    1010mm  Střed ramena
    //    2.  dock  995mm     od zdi
    //    3.  dock  1190mm    od zdi
    //    4.  dock  1385mm    od zdi
    //    5.  dock  1580mm    od zdi
    //    6.  dock  1775mm    od zdi
    //    7.  dock  1970mm    od zdi
    //    8.  dock  2165mm    od zdi

/*****************************************************************************************************************************/

//PIXY2 KAMERA

//Globální instance PIXY2 kamery
Pixy2 pixy;

//Různé funkce pixie kamery jako vzor
// pixy.ccc.getBlocks();               // Získání bloků z Pixy2 kamery


// pixy.ccc.numBlocks;                 // Počet detekovaných bloků


// pixy.ccc.blocks[i]                  // Přístup k jednotlivým blokům (i = index bloku)

// pixy.ccc.blocks[i].m_width;         // Šířka bloku
// pixy.ccc.blocks[i].m_height;        // Výška bloku

// pixy.ccc.blocks[i].m_x;             // X pozice bloku
// pixy.ccc.blocks[i].m_y;             // Y pozice bloku

// pixy.ccc.blocks[i].m_xTop;         // X pozice horního okraje bloku
// pixy.ccc.blocks[i].m_yTop;         // Y pozice horního okraje bloku
// pixy.ccc.blocks[i].m_xBottom;      // X pozice dolního okraje bloku
// pixy.ccc.blocks[i].m_yBottom;      // Y pozice dolního okraje bloku

// pixy.ccc.blocks[i].m_signature;     // Barva bloku (1 = červená, 2 = modrá, atd.)
//
// pixy.ccc.blocks[i].m_angle;         // Úhel bloku (pokud je relevantní)
// pixy.ccc.blocks[i].m_index;         // Index bloku (pro identifikaci)
// pixy.ccc.blocks[i].m_age;           // Věk bloku (jak dlouho byl detekován)
// pixy.ccc.blocks[i].m_xSpeed;        // Rychlost bloku v ose X
// pixy.ccc.blocks[i].m_ySpeed;        // Rychlost bloku v ose Y
// pixy.ccc.blocks[i].m_xAccel;       // Zrychlení bloku v ose X
// pixy.ccc.blocks[i].m_yAccel;       // Zrychlení bloku v ose Y

/*****************************************************************************************************************************/


/***********************************************************************************************************************/

//DOCKS CLASSES//

// Enum pro vlastní barvy tlačítek nebo bloků
enum class MyColor {
    NONE,
    RED,
    BLUE
};

    // Enum pro barvu docks
enum class Color {
    NON,
    RED,
    BLUE
};

    // Enum pro status docks
enum class Status {
    FILLED,
    EMPTY,
    ENEMY
};

    // Třída reprezentující jeden dock
class Dock {
private:
    int absolute_pos; // vzdálenost od zdi v mm
    Color color;
    Status status;

public:
    // Konstruktor
    Dock(int pos = 0, Color c = Color::NON, Status s = Status::EMPTY)
        : absolute_pos(pos), color(c), status(s) {}

    // Gettery a settery
    int getAbsolutePos() const { return absolute_pos; }
    void setAbsolutePos(int pos) { absolute_pos = pos; }

    Color getColor() const { return color; }
    void setColor(Color c) { color = c; }

    Status getStatus() const { return status; }
    void setStatus(Status s) { status = s; }

    // Pomocné metody pro výpis
    std::string colorToString() const {
        switch(color) {
            case Color::NON: return "NON";
            case Color::RED: return "RED";
            case Color::BLUE: return "BLUE";
            default: return "UNKNOWN";
        }
    }

    std::string statusToString() const {
        switch(status) {
            case Status::FILLED: return "FILLED";
            case Status::EMPTY: return "EMPTY";
            case Status::ENEMY: return "ENEMY";
            default: return "UNKNOWN";
        }
    }

    // Výpis informací o docku
    void printInfo() const {
        std::cout << "Dock: Position=" << absolute_pos << "mm, Color=" 
                  << colorToString() << ", Status=" << statusToString() << std::endl;
    }
};

    // Třída pro správu všech docks
class DockManager {
private:
    std::vector<Dock> docks;
    const int DOCK_COUNT = 8;

public:
    // Konstruktor - inicializuje 8 docks
    DockManager() {
        docks.reserve(DOCK_COUNT);
        for (int i = 0; i < DOCK_COUNT; ++i) {
            docks.emplace_back(0, Color::NON, Status::EMPTY);
        }
    }

    // Přístup k jednotlivým docks
    Dock& getDock(int index) {
        if (index < 0 || index >= DOCK_COUNT) {
            throw std::out_of_range("Invalid dock index");
        }
        return docks[index];
    }

    const Dock& getDock(int index) const {
        if (index < 0 || index >= DOCK_COUNT) {
            throw std::out_of_range("Invalid dock index");
        }
        return docks[index];
    }

    // Výpis informací o všech docích
    void printAllDocks() const {
        for (int i = 0; i < DOCK_COUNT; ++i) {
            std::cout << "Dock " << i << ": ";
            docks[i].printInfo();
        }
    }

    // Počet docks
    int getDockCount() const {
        return DOCK_COUNT;
    }

    int findNearestEmptyDock(int reference_pos) const {
        int nearest_index = -1;
        int min_distance = INT_MAX;
        
        for (int i = 0; i < DOCK_COUNT; ++i) {
            if (docks[i].getStatus() == Status::EMPTY) {
                int distance = abs(docks[i].getAbsolutePos() - reference_pos);
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest_index = i;
                }
            }
        }
        
        return nearest_index;
    }

    /*
     * Vrátí absolutní pozici požadovaného docku
     * @param dock_index Index docku (0-7)
     * @return Absolutní pozice v mm od začátku
     * @throws std::out_of_range Pokud je index mimo rozsah
     */

    int getDockAbsolutePosition(int dock_index) const {
        if (dock_index < 0 || dock_index >= DOCK_COUNT) {
            throw std::out_of_range("Neplatný index docku");
        }
        return docks[dock_index].getAbsolutePos();
    }
};

// Globální instance dock manageru
DockManager manager;

/***********************************************************************************************************************/

// SLEDOVÁNÍ POZICE ROBOTA

// Třída pro sledování aktuální pozice robota
//      Sleduje aktuální pozici robota v mm od startovní pozice
//      Umožňuje inicializaci, aktualizaci a resetování pozice
//      Umožňuje získání aktuální pozice robota

class PositionTracker {
private:
    int current_position; // aktuální pozice v mm od startu
    bool is_initialized;

public:
    PositionTracker() : current_position(0), is_initialized(false) {}

    // Inicializace pozice robota
    void initialize(int start_position = 65) {
        current_position = start_position;
        is_initialized = true;
    }

    // Aktualizace pozice po pohybu
    void updatePosition(int movement_mm) {
        if (!is_initialized) {
            Serial.println("Warning: PositionTracker not initialized!");
            initialize();
        }
        current_position += movement_mm;
    }

    // Získání aktuální pozice
    int getCurrentPosition() const {
        if (!is_initialized) {
            Serial.println("Warning: PositionTracker not initialized!");
        }
        return current_position;
    }

    // Reset pozice
    void reset(int new_position = 0) {
        current_position = new_position;
        is_initialized = true;
    }
};

// Globální instance sledovače pozice
PositionTracker positionTracker;

/***********************************************************************************************************************/

//DETEKCE SOUPEŘE

//Ultrazvuky


/***********************************************************************************************************************/

// // PID řízení pro pohyb robota

// float M_wheel_circumference = 65.0f * PI; // Průměr kola v mm * PI

// int32_t MmToTicks(float mm){ return (mm / M_wheel_circumference) * 38.55937f * 48.f; }

// float TicksToMm(int32_t ticks) { return float(ticks) / 38.55937f / 48.f * M_wheel_circumference; }

// int Odchylka = 0, Integral = 0, Last_odchylka = 0;
// void Check_PID(int power, int M1_pos, int M4_pos){
//         int Max_integral = 1000;
//         Odchylka = M1_pos - M4_pos;
//         Integral += Odchylka;
//         if (Integral >  Max_integral) Integral =  Max_integral;
//         if (Integral < -Max_integral) Integral = -Max_integral;
//         int rawPower = power + Odchylka * Kp + Integral * (Ki+2) + (Odchylka - Last_odchylka) * Kd;
//         // saturace
//         const int maxPower = 32000;
//         if      (rawPower >  maxPower) rawPower =  maxPower;
//         else if (rawPower < -maxPower) rawPower = -maxPower;
//         auto& man = rb::Manager::get();
//         Odchylka = M1_pos - M4_pos;
//         Integral += Odchylka;
//         man.motor(rb::MotorId::M1).power(-power * 0.9185f);//
//         man.motor(rb::MotorId::M4).power(rawPower);
//         Last_odchylka = Odchylka;
// }

// int M4_pos = 0;
// int M1_pos = 0;


// std::atomic<bool> movingToDock(false);

// void moveToDockAsync(Dock targetDock) {
//     movingToDock = true;

//     // Parametry pro pohyb
//     float maxSpeed = 100; // [mm/s]
//     float acceleration = 50; // [mm/s^2]
//     float slowDownDistance = 200; // [mm] před cílem začne zpomalovat

//     // Předpokládaná délka trasy
//     float distanceToDock = std::abs(robot.pos - targetDock.absolute_pos);
//     float currentSpeed = 0;

//     while (movingToDock) {
//         float currentPos = robot.pos; // Přečti aktuální pozici
//         float remainingDistance = std::abs(targetDock.absolute_pos - currentPos);

//         // 📦 Zjištění barvy doku pomocí PixyCam
//         for (int i = 0; i < pixy.ccc.numBlocks; i++) {
//             auto& block = pixy.ccc.blocks[i];
//             if (block.m_signature == RED_SIGNATURE) {
//                 // Nalezli jsme červený cíl
//             }
//             // můžeš porovnávat s targetDock.color == RED atd.
//         }

//         // 💥 Detekce překážky/soupeře
//         if (detectEnemyNearby()) {
//             log("Enemy detected! Stopping.");
//             robot.setSpeed(0);
//             break; // nebo počkej, obkruž atd.
//         }

//         // 🚀 Výpočet rychlosti: akcelerace / zpomalení
//         if (remainingDistance < slowDownDistance) {
//             // Zpomalení
//             currentSpeed = std::min(currentSpeed, remainingDistance / slowDownDistance * maxSpeed);
//         } else {
//             // Zrychlení
//             currentSpeed = std::min(currentSpeed + acceleration * 0.05f, maxSpeed); // 0.05s smyčka
//         }

//         // 🔁 Nastav rychlost robota
//         if (targetDock.absolute_pos > currentPos) {
//             robot.setSpeed(currentSpeed);
//         } else {
//             robot.setSpeed(-currentSpeed);
//         }

//         // 🎯 Konec pohybu
//         if (remainingDistance < 5) {
//             robot.setSpeed(0);
//             movingToDock = false;
//             log("Dock reached.");
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     }
// }
// void xxx(int distance, int speed){
    

//     rkMotorsDriveAsync(distance, distance, speed, speed, []() { Serial.println("Pohyb dokončen."); });

//     rkMotorsGetPositionById(rb::MotorId::M1, [&](int pos) {
//         M1_pos = pos;
//         Serial.printf("M1 aktuální pozice: %d\n", M1_pos);
//     });
//         rkMotorsGetPositionById(rb::MotorId::M4, [&](int pos) {
//         M1_pos = pos;
//         Serial.printf("M1 aktuální pozice: %d\n", M1_pos);
//     });
// }

// struct MoveToDockArgs {
//     int dockIndex;
//     float maxSpeed;
//     std::function<void()> onArrived;
// };

// // Asynchronní pohybová funkce s callbackem po dokončení
// void moveToDockAsync(int dockIndex, float maxSpeed, std::function<void()> onArrived) {
//     // Vytvoříme nový FreeRTOS task
//     xTaskCreate(
//         [](void* param) {
//             MoveToDockArgs* args = static_cast<MoveToDockArgs*>(param);

//             int dockPos = manager.getDockAbsolutePosition(args->dockIndex);
//             int robotPos = positionTracker.getCurrentPosition();
//             int distance = dockPos - robotPos;
//             int direction = (distance >= 0) ? 1 : -1;
//             float absDistance = abs(distance);

//             // Parametry akcelerace/decelerace
//             float accel = 0.5f; // mm/ms^2 (nastav dle potřeby)
//             float decel = 0.5f; // mm/ms^2
//             float speed = 0;
//             float dt = 20; // ms, perioda smyčky

//             float traveled = 0;
//             bool enemyDetected = false;

//             // Zrychlování
//             while (speed < args->maxSpeed && traveled < absDistance/2) {
//                 speed += accel * dt;
//                 if (speed > args->maxSpeed) speed = args->maxSpeed;
//                 float step = speed * dt / 1000.0f;
//                 traveled += step;
//                 positionTracker.updatePosition(direction * step);
//                 setMotorsPower(direction * speed, direction * speed);

//                 if (EmemyDetection()) { enemyDetected = true; break; }
//                 vTaskDelay(dt / portTICK_PERIOD_MS);
//             }

//             // Konstantní rychlost
//             while (traveled < absDistance - 100 && !enemyDetected) { // 100 mm před cílem začni zpomalovat
//                 float step = speed * dt / 1000.0f;
//                 traveled += step;
//                 positionTracker.updatePosition(direction * step);
//                 setMotorsPower(direction * speed, direction * speed);

//                 if (EnemyDetection()) { enemyDetected = true; break; }
//                 vTaskDelay(dt / portTICK_PERIOD_MS);
//             }

//             // Zpomalení
//             while (speed > 0 && !enemyDetected) {
//                 speed -= decel * dt;
//                 if (speed < 0) speed = 0;
//                 float step = speed * dt / 1000.0f;
//                 traveled += step;
//                 positionTracker.updatePosition(direction * step);
//                 setMotorsPower(direction * speed, direction * speed);

//                 if (EmemyDetection()) { enemyDetected = true; break; }
//                 vTaskDelay(dt / portTICK_PERIOD_MS);
//             }

//             setMotorsPower(0, 0);

//             // Po dojetí/do zastavení
//             if (args->onArrived && !enemyDetected) args->onArrived();

//             delete args;
//             vTaskDelete(NULL);
//         },
//         "MoveToDockAsync",
//         4096,
        
//         new MoveToDockArgs{dockIndex, maxSpeed, onArrived}, // ← zde použij pojmenovanou strukturu
//         1,
//         nullptr
//     );
// }

/***********************************/

// struct NavigationTask {
//     int targetDockIndex;
//     float maxSpeed;  // mm/s
//     float accel;     // mm/s^2
//     std::function<void(bool, Color)> onFinish;
// };

// void moveToDockAsync(int dockIndex, float speed, std::function<void(bool, Color)> callback) {
//     // Vytvoření struktury s parametry pro task
//     NavigationTask* taskParams = new NavigationTask{
//         .targetDockIndex = dockIndex,
//         .maxSpeed = speed,
//         .accel = speed / 2.0f, // Rozumná výchozí akcelerace
//         .onFinish = callback
//     };

//     // Vytvoření FreeRTOS tasku
//     xTaskCreate(
//         [](void* params) {
//             NavigationTask* task = static_cast<NavigationTask*>(params);
//             auto& man = rb::Manager::get();
            
//             // 1. Získání cílové pozice
//             int targetPos = manager.getDockAbsolutePosition(task->targetDockIndex);
//             int startPos = positionTracker.getCurrentPosition();
//             int distance = targetPos - startPos;
//             int direction = (distance > 0) ? 1 : -1;
            
//             // 2. Inicializace motorů a proměnných
//             man.motor(rb::MotorId::M1).setCurrentPosition(0);
//             man.motor(rb::MotorId::M4).setCurrentPosition(0);
            
//             float currentSpeed = 0;
//             float currentPos = startPos;
//             bool enemyDetected = false;
//             Color dockColor = Color::NON;
            
//             // 3. Hlavní pohybová smyčka
//             while (abs(currentPos - startPos) < abs(distance) && !enemyDetected) {
//                 // 3.1 Aktualizace pozice a rychlosti
//                 int M1_pos = 0, M4_pos = 0;
//                 man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) { M1_pos = -info.position(); });
//                 man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) { M4_pos = info.position(); });
                
//                 // Přepočet pozice encoderů na mm a aktualizace trackingu
//                 float moved = ((M1_pos + M4_pos) / 2.0f) * (127.5 * PI) / (40.4124852f * 48.f);
//                 currentPos = startPos + (direction * moved);
//                 positionTracker.updatePosition(static_cast<int>(currentPos - positionTracker.getCurrentPosition()));
                
//                 // 3.2 Řízení rychlosti (akcelerace/decelerace)
//                 float remainingDistance = abs(distance) - abs(currentPos - startPos);
                
//                 // Fáze zrychlování
//                 if (currentSpeed < task->maxSpeed && remainingDistance > 200) {
//                     currentSpeed += task->accel * 0.02f; // 20ms časový krok
//                     if (currentSpeed > task->maxSpeed) currentSpeed = task->maxSpeed;
//                 }
//                 // Fáze zpomalování
//                 else if (remainingDistance < 200) {
//                     currentSpeed -= task->accel * 0.02f;
//                     if (currentSpeed < 0) currentSpeed = 0;
//                 }
                
//                 // 3.3 Detekce soupeře
//                 if (EnemyDetection()) {
//                     enemyDetected = true;
//                     break;
//                 }
                
//                 // 3.4 Detekce barvy docku při přiblížení
//                 if (remainingDistance < 150 && dockColor == Color::NON) {
//                     pixy.ccc.getBlocks();
//                     if (pixy.ccc.numBlocks > 0) {
//                         if (pixy.ccc.blocks[0].m_signature == 1) {
//                             dockColor = Color::RED;
//                             manager.getDock(task->targetDockIndex).setColor(Color::RED);
//                         } 
//                         else if (pixy.ccc.blocks[0].m_signature == 2) {
//                             dockColor = Color::BLUE;
//                             manager.getDock(task->targetDockIndex).setColor(Color::BLUE);
//                         }
//                     }
//                 }
                
//                 // 3.5 PID řízení motorů
//                 int odchylka = M1_pos - M4_pos;
//                 int power = static_cast<int>(currentSpeed * 32000 / 100); // Převod na rozsah motoru
                
//                 man.motor(rb::MotorId::M1).power(-direction * power * 0.92f);
//                 int powerM4 = direction * power + odchylka * Kp;
//                 man.motor(rb::MotorId::M4).power(powerM4);
                
//                 vTaskDelay(20 / portTICK_PERIOD_MS);
//             }
            
//             // 4. Zastavení motorů
//             man.motor(rb::MotorId::M1).power(0);
//             man.motor(rb::MotorId::M4).power(0);
            
//             // 5. Volání callbacku s výsledkem
//             if (task->onFinish) {
//                 task->onFinish(!enemyDetected, dockColor);
//             }
            
//             // 6. Úklid
//             delete task;
//             vTaskDelete(NULL);
//         },
//         "DockNavigation",  // Název tasku
//         4096,              // Velikost zásobníku
//         taskParams,        // Parametry
//         1,                // Priorita
//         nullptr           // Handle
//     );
// }

/***********************************/

// void moveStraight_with_anotherTask(int distance){
    
//     auto& man = rb::Manager::get(); // vytvoří referenci na man class
    
//     man.motor(rb::MotorId::M1).setCurrentPosition(0);
//     man.motor(rb::MotorId::M4).setCurrentPosition(0);
    
//     //otevreni_prepazky(); // Otevře prepážku

//     man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) { M4_pos = info.position(); });
//     man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) { M1_pos = -info.position(); });

//     //xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);

//     Serial.printf("Musí ujet::: M1_pos: %d mm, M4_pos: %d mm\n", MmToTicks(distance), MmToTicks(distance));
    
//     setMotorsPower(20000, 20000); // Nastaví motory na 20000

//     // Čekání na dosažení cílové vzdálenosti ale další vlákno bude pokračovat v běhu
//     while(M4_pos < MmToTicks(distance) && M1_pos < MmToTicks(distance)) {
//           man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) { M4_pos = info.position(); });
//           man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) { M1_pos = -info.position(); });

//           Serial.printf("[ENCODERY] M4_pos: %d\n", M4_pos);

//           if(M1_pos > 200 && M4_pos > 200) { Check_PID(20000, M1_pos, M4_pos); }

//           if(EnemyDetection()) { break; }
//           delay(10);
//     }

//     man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) { M4_pos = info.position(); });
//     man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) { M1_pos = -info.position(); });

//     Serial.printf("[ENCODERY] M4_pos: %d, M1_pos: %d\n", M4_pos, M1_pos);

//     //Aktualizace pozice robota
//     positionTracker.updatePosition(static_cast<int>(TicksToMm(M1_pos))); 
      
//     //vTaskDelete(chytejPukyHandle);
//     //chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
      
//     setMotorsPower(0, 0); // Zastaví motory
//     delay(100);

//     // Resetování pozic motorů
//     man.motor(rb::MotorId::M1).setCurrentPosition(0);
//     man.motor(rb::MotorId::M4).setCurrentPosition(0);
      
//     M1_pos = 0;
//     M4_pos = 0;
// }

// // Upravená funkce pro pohyb dopředu s aktualizací pozice robota
// void move_straight_with_tracking(float mm, float speed) {
//     forward(mm, speed); // původní funkce pro pohyb
    
//     if(speed < 0) mm *= -1;  // pokud je rychlost záporná, invertujeme vzdálenost

//     // Aktualizace pozice - pohyb dopředu přičítá vzdálenost
//     positionTracker.updatePosition(static_cast<int>(mm));
    
//     Serial.printf("Pohyb dopredu o %.1f mm. Aktualni pozice: %d mm\n", 
//                  mm, positionTracker.getCurrentPosition());
// }

// // Funkce pro získání aktuální pozice
// int getRobotPosition() {
//     return positionTracker.getCurrentPosition();
// }

// void navigateToDock(int dockIndex) {
//     try {
//         // Získání pozice docku
//         int dockPos = manager.getDockAbsolutePosition(dockIndex);
        
//         // Získání aktuální pozice robota
//         int robotPos = positionTracker.getCurrentPosition();
        
//         // Výpočet potřebné vzdálenosti
//         int distance = dockPos - robotPos;
        
//         if (distance > 0) {
//             // Robot k docku jede dopředu
//             move_straight_with_tracking(static_cast<float>(abs(distance)), 40.0f); // Pohyb vpřed k docku
//         } else if (distance < 0) {
//             //Robot k docku couvá
//             move_straight_with_tracking(static_cast<float>(abs(distance)), -40.0f); // Pohyb vzad k docku
//         } else {
//             // Robot je již na správné pozici
//             Serial.println("Jsme u cílového docku!");
//             rkLedGreen(true); // Zapnutí zelené LED
//         }
        
//         //Naložení baterie do docku
//         //load_dock();

//         // Označení docku jako obsazený
//         manager.getDock(dockIndex).setStatus(Status::FILLED);
        
//     } catch (const std::out_of_range& e) {
//         Serial.println("Chyba: Neplatný index docku!");
//     }
// }

// void goToAbsolutePosition(int absPos) {

//     // Získání aktuální pozice robota
//     int robotPos = positionTracker.getCurrentPosition();
            
//     // Výpočet potřebné vzdálenosti
//     int distance = absPos - robotPos;
            
//     if (distance > 0) {
//         // Robot k docku jede dopředu
//         move_straight_with_tracking(static_cast<float>(abs(distance)), 60.0f); // Pohyb vpřed k docku
//     } else if (distance < 0) {
//         //Robot k docku couvá
//         move_straight_with_tracking(static_cast<float>(abs(distance)), -60.0f); // Pohyb vzad k docku
//     } else {
//         // Robot je již na správné pozici
//         Serial.println("Jsme na cílové pozici!");
//         rkLedGreen(true); // Zapnutí zelené LED
//     }
// }
// Funkce pro detekci soupeře pomocí ultrazvukové věže
// bool EnemyDetection() {

// }

struct NavigationTask {
    int targetDockIndex;
    float maxSpeed;  // mm/s
    float accel;     // mm/s^2
    std::function<void(bool, Color)> onFinish;
};

void moveToDockAsync(int dockIndex, float speed, std::function<void(bool, Color)> callback) {
    struct NavigationTask {
        int targetDockIndex;
        float maxSpeed;  // mm/s
        float accel;     // mm/s^2
        std::function<void(bool, Color)> onFinish;
    };

    NavigationTask* taskParams = new NavigationTask{
        dockIndex,
        speed,
        speed / 2.0f,
        callback
    };

    xTaskCreate(
        [](void* params) {
            NavigationTask* task = static_cast<NavigationTask*>(params);
            auto& man = rb::Manager::get();

            int targetPos = manager.getDockAbsolutePosition(task->targetDockIndex);
            int startPos = positionTracker.getCurrentPosition();
            int distance = targetPos - startPos;
            int direction = (distance > 0) ? 1 : -1;
            distance = abs(distance);

            man.motor(rb::MotorId::M1).setCurrentPosition(0);
            man.motor(rb::MotorId::M4).setCurrentPosition(0);

            float currentSpeed = 0;
            float traveled = 0;
            bool enemyDetected = false;
            Color dockColor = Color::NON;

            // Sdílené proměnné pro enkodéry
            volatile int M1_pos = 0;
            volatile int M4_pos = 0;

            while (traveled < distance && !enemyDetected) {
                // Asynchronní čtení pozic motorů
                man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) { M1_pos = abs(info.position()); });
                man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) { M4_pos = abs(info.position()); });

                vTaskDelay(10 / portTICK_PERIOD_MS); // Dej callbackům čas

                float avgPos = (M1_pos + M4_pos) / 2.0f;
                traveled = avgPos * (127.5 * PI) / (40.4124852f * 48.f);
                Serial.printf("Ujeto: %.2f mm, Cílová pozice: %d mm\n", traveled, targetPos);

                int currentAbsolutePos = startPos + (direction * traveled);
                positionTracker.updatePosition(currentAbsolutePos - positionTracker.getCurrentPosition());

                float remainingDistance = distance - traveled;

                // Akcelerace/decelerace
                if (currentSpeed < task->maxSpeed && remainingDistance > 200) {
                    currentSpeed = std::min(currentSpeed + task->accel * 0.02f, task->maxSpeed);
                } else if (remainingDistance < 200) {
                    currentSpeed = std::max(currentSpeed - task->accel * 0.02f, 0.0f);
                }

                // Detekce soupeře
                if (EnemyDetection()) {
                    enemyDetected = true;
                    break;
                }

                // Detekce barvy docku při přiblížení
                if (remainingDistance < 150 && dockColor == Color::NON) {
                    pixy.ccc.getBlocks();
                    if (pixy.ccc.numBlocks > 0) {
                        dockColor = (pixy.ccc.blocks[0].m_signature == 1) ? Color::RED : 
                                   (pixy.ccc.blocks[0].m_signature == 2) ? Color::BLUE : Color::NON;
                    }
                }

                // PID řízení motorů
                int odchylka = M1_pos - M4_pos;
                int power = static_cast<int>(currentSpeed * 32000 / 100);

                man.motor(rb::MotorId::M1).power(-direction * power * 0.92f);
                int powerM4 = direction * power + odchylka * Kp;
                powerM4 = constrain(powerM4, -32000, 32000);
                man.motor(rb::MotorId::M4).power(powerM4);

                vTaskDelay(20 / portTICK_PERIOD_MS);
            }

            man.motor(rb::MotorId::M1).power(0);
            man.motor(rb::MotorId::M4).power(0);

            positionTracker.updatePosition(targetPos - positionTracker.getCurrentPosition());

            if (task->onFinish) {
                task->onFinish(!enemyDetected, dockColor);
            }

            delete task;
            vTaskDelete(NULL);
        },
        "DockNavigation",
        4096,
        taskParams,
        1,
        nullptr
    );
}

struct AbsMoveTask {
    int targetPos;
    float maxSpeed;
    float accel;
    std::function<void(bool)> onFinish;
};

void moveToAbsolutePositionAsync(int absTargetPos, float speed, std::function<void(bool)> callback) {
    xTaskCreate(
        [](void* params) {
            struct {
                int targetPos;
                float maxSpeed;
                float accel;
                std::function<void(bool)> onFinish;
            } *task = static_cast<decltype(task)>(params);

            auto& man = rb::Manager::get();
            int startPos = positionTracker.getCurrentPosition();
            int distance = task->targetPos - startPos;
            int direction = (distance > 0) ? 1 : -1;
            distance = abs(distance);

            man.motor(rb::MotorId::M1).setCurrentPosition(0);
            man.motor(rb::MotorId::M4).setCurrentPosition(0);

            float currentSpeed = 0;
            float traveled = 0;
            bool enemyDetected = false;

            volatile int M1_pos = 0;
            volatile int M4_pos = 0;

            while (traveled < distance && !enemyDetected) {
                man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) { M1_pos = abs(info.position()); });
                man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) { M4_pos = abs(info.position()); });
                vTaskDelay(10 / portTICK_PERIOD_MS);

                float avgPos = (M1_pos + M4_pos) / 2.0f;
                traveled = avgPos * (127.5 * PI) / (40.4124852f * 48.f);

                int currentAbsolutePos = startPos + (direction * traveled);
                positionTracker.updatePosition(currentAbsolutePos - positionTracker.getCurrentPosition());

                float remainingDistance = distance - traveled;
                if (currentSpeed < task->maxSpeed && remainingDistance > 200) {
                    currentSpeed = std::min(currentSpeed + task->accel * 0.02f, task->maxSpeed);
                } else if (remainingDistance < 200) {
                    currentSpeed = std::max(currentSpeed - task->accel * 0.02f, 0.0f);
                }

                if (EnemyDetection()) {
                    enemyDetected = true;
                    break;
                }

                int odchylka = M1_pos - M4_pos;
                int power = static_cast<int>(currentSpeed * 32000 / 100);

                man.motor(rb::MotorId::M1).power(-direction * power * 0.92f);
                int powerM4 = direction * power + odchylka * Kp;
                powerM4 = constrain(powerM4, -32000, 32000);
                man.motor(rb::MotorId::M4).power(powerM4);

                vTaskDelay(20 / portTICK_PERIOD_MS);
            }

            man.motor(rb::MotorId::M1).power(0);
            man.motor(rb::MotorId::M4).power(0);

            positionTracker.updatePosition(task->targetPos - positionTracker.getCurrentPosition());

            if (task->onFinish) {
                task->onFinish(!enemyDetected);
            }

            delete task;
            vTaskDelete(NULL);
        },
        "AbsMove",
        4096,
        new decltype(*((decltype(nullptr))nullptr)){absTargetPos, speed, speed / 2.0f, callback},
        1,
        nullptr
    );
}

// void moveToDockAsync(int dockIndex, float speed, std::function<void(bool, Color)> callback) {
//     NavigationTask* taskParams = new NavigationTask{
//         .targetDockIndex = dockIndex,
//         .maxSpeed = speed,
//         .accel = speed / 2.0f,
//         .onFinish = callback
//     };

//     xTaskCreate(
//         [](void* params) {
//             NavigationTask* task = static_cast<NavigationTask*>(params);
//             auto& man = rb::Manager::get();
            
//             // 1. Získání cílové pozice
//             int targetPos = manager.getDockAbsolutePosition(task->targetDockIndex);
//             int startPos = positionTracker.getCurrentPosition();
//             int distance = targetPos - startPos;
//             int direction = (distance > 0) ? 1 : -1;
//             distance = abs(distance); // Používáme absolutní hodnotu
            
//             // 2. Inicializace motorů
//             man.motor(rb::MotorId::M1).setCurrentPosition(0);
//             man.motor(rb::MotorId::M4).setCurrentPosition(0);
            
//             float currentSpeed = 0;
//             float traveled = 0;
//             bool enemyDetected = false;
//             Color dockColor = Color::NON;
            
//             // 3. Hlavní pohybová smyčka
//             while (traveled < distance && !enemyDetected) {
//                 // 3a. Čtení pozic encoderů
//                 int M1_pos = 0, M4_pos = 0;
//                 man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) { M1_pos = abs(info.position()); });
//                 man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) { M4_pos = abs(info.position()); });
                
//                 // 3b. Výpočet ujeté vzdálenosti v mm
//                 float avgPos = (M1_pos + M4_pos) / 2.0f;
//                 traveled = avgPos * (127.5 * PI) / (40.4124852f * 48.f);
//                 Serial.printf("Ujeto: %.2f mm, Cílová pozice: %d mm\n", traveled, targetPos);
                
//                 // 3c. Aktualizace absolutní pozice
//                 int currentAbsolutePos = startPos + (direction * traveled);
//                 positionTracker.updatePosition(currentAbsolutePos - positionTracker.getCurrentPosition());
                
//                 // 3d. Řízení rychlosti
//                 float remainingDistance = distance - traveled;
                
//                 // Zrychlování
//                 if (currentSpeed < task->maxSpeed && remainingDistance > 200) {
//                     currentSpeed = min(currentSpeed + task->accel * 0.02f, task->maxSpeed);
//                 }
//                 // Zpomalování
//                 else if (remainingDistance < 200) {
//                     currentSpeed = max(currentSpeed - task->accel * 0.02f, 0.0f);
//                 }
                
//                 // 3e. Detekce soupeře
//                 if (EnemyDetection()) {
//                     enemyDetected = true;
//                     break;
//                 }
                
//                 // 3f. Detekce barvy (pouze když jsme blízko)
//                 if (remainingDistance < 150 && dockColor == Color::NON) {
//                     pixy.ccc.getBlocks();
//                     if (pixy.ccc.numBlocks > 0) {
//                         dockColor = (pixy.ccc.blocks[0].m_signature == 1) ? Color::RED : 
//                                   (pixy.ccc.blocks[0].m_signature == 2) ? Color::BLUE : Color::NON;
//                     }
//                 }
                
//                 // 3g. Řízení motorů
//                 int odchylka = M1_pos - M4_pos;
//                 int power = static_cast<int>(currentSpeed * 32000 / 100);
                
//                 man.motor(rb::MotorId::M1).power(-direction * power * 0.92f);
//                 int powerM4 = direction * power + odchylka * Kp;
//                 powerM4 = constrain(powerM4, -32000, 32000);
//                 man.motor(rb::MotorId::M4).power(powerM4);
                
//                 vTaskDelay(20 / portTICK_PERIOD_MS);
//             }
            
//             // 4. Zastavení motorů
//             man.motor(rb::MotorId::M1).power(0);
//             man.motor(rb::MotorId::M4).power(0);
            
//             // 5. Korekce pozice po dojezdu
//             positionTracker.updatePosition(targetPos - positionTracker.getCurrentPosition());
            
//             // 6. Callback
//             if (task->onFinish) {
//                 task->onFinish(!enemyDetected, dockColor);
//             }
            
//             delete task;
//             vTaskDelete(NULL);
//         },
//         "DockNavigation",
//         4096,
//         taskParams,
//         1,
//         nullptr
//     );
// }

/*****************************************************************************************************************************/

//OVLÁDÍNÍ RAMENE

class RamenoController {
private:

    float default_speed; // Výchozí rychlost pro serva

public:

    RamenoController() : default_speed(80) {}

    void set_defaultSmartServosSpeed(float speed)   { default_speed = speed; }


    int get_defaultSmartServosSpeed() const{ return default_speed; }


    void Magnet(bool on)            { if (on) { rkServosSetPosition(1, 90); } else { rkServosSetPosition(1, 0); } }    // 90 - drží baterii, 0 - pustí baterii
    
    void Up(float speed = 80)       { auto &bus = rkSmartServoBus(2); s_s_move(bus, 0, 50, speed); }        // Rameno - nahoře

    void Down(float speed = 80)     { auto &bus = rkSmartServoBus(2); s_s_move(bus, 0, 130, speed); }       // Rameno - dole

    void Active(float speed = 80)   { auto &bus = rkSmartServoBus(2); s_s_move(bus, 0, 60, speed); }        // Rameno - nahoře - aktiv

    void Left(float speed = 80)     { auto &bus = rkSmartServoBus(2); s_s_move(bus, 1, 35, speed); }        // Rameno - levá

    void Right(float speed = 80)    { auto &bus = rkSmartServoBus(2); s_s_move(bus, 1, 215, speed); }       // Rameno - pravá

    void Center(float speed = 80)   { auto &bus = rkSmartServoBus(2); s_s_move(bus, 1, 125, speed); }       // Rameno - střed

    void getColorPosition(float speed = 80) { auto &bus = rkSmartServoBus(2); s_s_move(bus, 0, 80, speed); } 
        
        
        
        //int dockIndex, Color color
        
        // // Získání absolutní pozice docku
        // int dockPos = manager.getDockAbsolutePosition(dockIndex);
        
        // // Nastavení barvy docku
        // manager.getDock(dockIndex).setColor(color);
        
        // // Výpis informace o barvě a pozici docku
        // Serial.printf("Dock %d má barvu %s a je na pozici %d mm od zdi.\n", 
        //               dockIndex, manager.getDock(dockIndex).colorToString().c_str(), dockPos);
    

    // Funkce pro naložení baterie na rameno
    void load_battery(int WaiterIndex, float speed = 80) {
        delay(100);
        Up(speed);
        Magnet(true);
        delay(WaiterIndex);
        Left(speed);
        delay(WaiterIndex);
        Down(speed);
        delay(WaiterIndex);
        Active(speed);
        delay(WaiterIndex);
        Center(speed);
        delay(WaiterIndex);
    }

    // Funkce pro vyložení baterie z ramene
    void unload_battery(int dockIndex, int WaiterIndex, float speed = 80) {
        delay(100);
        Active(speed);
        delay(WaiterIndex);
        Right(speed);
        delay(WaiterIndex);
        Down(speed);
        delay(WaiterIndex);
        Magnet(false);
        delay(WaiterIndex);
        Up(speed);
        delay(WaiterIndex); 
        Magnet(true);
        Center(speed);
        delay(WaiterIndex);

        // Označení docku jako obsazený
        manager.getDock(dockIndex).setStatus(Status::FILLED);
    }

    // Funkce pro naložení baterie a rovnou ji přesunout do docku
    void load_dock(int dockIndex, int WaiterIndex, float speed = 80) {
        delay(10);
        Up(speed);
        Magnet(true);
        delay(WaiterIndex);
        Left(speed);
        delay(WaiterIndex);
        Down(speed);
        delay(WaiterIndex);
        Active(speed);
        delay(WaiterIndex);
        Right(speed);
        delay(2 * WaiterIndex);
        Down(speed);
        delay(WaiterIndex);
        Magnet(false);
        //delay(WaiterIndex);
        Up(speed);
        delay(WaiterIndex); 
        Magnet(true);
        Center(speed);
        delay(WaiterIndex);

        // Označení docku jako obsazený
        manager.getDock(dockIndex).setStatus(Status::FILLED);
    }

};

// Globální instance RamenoController
RamenoController Rameno;


/*****************************************************************************************************************************/
/*****************************************************************************************************************************/

// Hlavní setup funkce
// Inicializuje Robotku, nastaví piny a inicializuje docks
// Nastaví sledování pozice robota a inicializuje docks s jejich absolutními pozicemi
// Připraví serva a LED diody
// Nastaví tlačítka pro ovládání robota
// Připraví správce docks a nastaví jejich absolutní pozice
void setup() {

    //Robotka settings
    rkConfig cfg;
    rkSetup(cfg);

    Serial.begin(115200);

    printf("Robotka started!\n");
    
    // Nastavení pinů pro tlačítka
    pinMode(Bbutton1, INPUT_PULLUP);
    pinMode(Bbutton2, INPUT_PULLUP);
    
    //Nastavení smart servos
    auto &bus = rkSmartServoBus(2);
    s_s_init(bus, 1, 30, 220); // Servo 1   Min 30     Max 220
    s_s_init(bus, 0, 40, 135); // Servo 0   Min 40     Max 135

    //Nastavení Pixy2 kamery
    pixy.init();
    pixy.changeProg("color_connected_components"); 
    printf("Pixy2 kamera inicializována!\n");

    // Inicializace sledování pozice
    positionTracker.initialize(); // Startovní pozice 0 mm
    
    // Nastavení absolutních pozic pro docks:
    manager.getDock(0).setAbsolutePos(800);
    manager.getDock(1).setAbsolutePos(995);
    manager.getDock(2).setAbsolutePos(1190);
    manager.getDock(3).setAbsolutePos(1385);
    manager.getDock(4).setAbsolutePos(1580);
    manager.getDock(5).setAbsolutePos(1775);
    manager.getDock(6).setAbsolutePos(1970);
    manager.getDock(7).setAbsolutePos(2165);

    //Nastavení ramene do výchozí pozice
    Rameno.Magnet(true); // Zapnutí magnetu
    Rameno.set_defaultSmartServosSpeed(80); // Nastavení výchozí rychlosti pro serva
    Rameno.Up();    // Rameno - nahoru
    Rameno.Center(); // Rameno - střed
    
    rkLedRed(true); 
    rkLedBlue(true); 
    rkLedGreen(false);
    rkLedYellow(false);
    
    delay(1000); // Počkáme 1 sekundu, aby se vše inicializovalo
    //move.Straight(500, 2000, 5000, 0); 
    moveToDockAsync(0, 60.0f, [](bool success, Color color) {});
    delay(5000); // Počkáme 1 sekundu, aby se vše inicializovalo


    moveToAbsolutePositionAsync(0, 60.0f, [](bool success) {});
    //move.Stop(); // Zastavení robota

}
  int pos = 50;
/*****************************************************************************************************************************/
/*****************************************************************************************************************************/

// Hlavní smyčka programu
void loop() {

//     auto &bus = rkSmartServoBus(2);
    
//     //Left button BLUE // Nastavení barvy na modrou
//     if ((digitalRead(Bbutton1) == LOW)) { MyColor color = MyColor::BLUE; }
    
//     //Right button RED // Nastavení barvy na červenou
//     if ((digitalRead(Bbutton2) == LOW)) { MyColor color = MyColor::RED; }

      


//     if (rkButtonIsPressed(BTN_UP)) {
    
//         // moveToDockAsync(0, 60.0f, [](bool success, Color color) {
//         //     if (success) {
//         //         Serial.println("Úspěšně dorazili k docku 0!");
//         //         rkLedGreen(true); // Zapnutí zelené LED
//         //     } else {
//         //         Serial.println("Detekován soupeř při cestě k docku 0.");
//         //         rkLedRed(true); // Zapnutí červené LED
//         //     }
//         //     //Serial.printf("Barva docku: %s\n", (color).c_str());
//         // });
//     }
//   if (rkButtonIsPressed(BTN_DOWN)) {
//         // move_straight_with_tracking(200, 40);
//         // goToAbsolutePosition(0);
//         printf("Kouká se\n");
//         pixy.ccc.getBlocks();               // Získání bloků z Pixy2 kamery
//         printf("Počet Blocků %d inicializováno\n", pixy.ccc.numBlocks);

//         printf("Počet bloků: %d\n", pixy.ccc.numBlocks); // Výpis počtu detekovaných bloků

//         if(pixy.ccc.blocks[0].m_signature == 1) { // Pokud je blok červený
//             printf("Červený blok detekován!\n");
//             // Zde můžete přidat další logiku pro zpracování červeného bloku
//         } else if(pixy.ccc.blocks[0].m_signature == 2) { // Pokud je blok modrý
//             printf("Modrý blok detekován!\n");
//             // Zde můžete přidat další logiku pro zpracování modrého bloku
//         } else {
//             printf("Blok s jinou barvou detekován!\n");
//         }



//         // pixy.ccc.blocks[i].m_signature;     // Barva bloku (1 = červená, 2 = modrá, atd.)

//         // Procházení detekovaných bloků a jejich výpis
//         // if (pixy.ccc.numBlocks > 0) {
//         //     for (int i = 0; i < pixy.ccc.numBlocks; i++) {
//         //         Serial.printf("Block %d: X=%d, Y=%d, Width=%d, Height=%d, Signature=%d\n", 
//         //                       i, pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y, 
//         //                       pixy.ccc.blocks[i].m_width, pixy.ccc.blocks[i].m_height, 
//         //                       pixy.ccc.blocks[i].m_signature);
//         //     }
//         // } else {
//         //     Serial.println("Žádné bloky nebyly detekovány.");
//         // }
//     }
//   if (rkButtonIsPressed(BTN_ON)) {
//     // move_straight_with_tracking(1000, 40);  // pohyb vpřed s trackingem
//     //    Rameno.load_battery(3000);
//   }
//   if (rkButtonIsPressed(BTN_OFF)) {
//         //positionTracker.initialize(); // Inicializace pozice robota na 70 mm
//         // navigateToDock(0);
//         // Rameno.load_dock(0, 3000); // Naložení baterie na rameno
//         // navigateToDock(3);
//         // navigateToDock(1);
//         // navigateToDock(1);


//         Rameno.load_dock(1, 1200, 100); // Naložení baterie do docku 1
//   }


//     // Pro jistotu, aby se cyklus neprováděl příliš rychle
//   delay(50);
}