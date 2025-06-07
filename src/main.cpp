//RBCX - Robotka
#include <Arduino.h>
#include "robotka.h"
#include "smart_servo_command.h"
#include "motor_commands.h"
#include "RBCXMotor.h"
#include "RBCX.h"

//#include "Movement.hpp"

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

using namespace lx16a; // aby nebylo třeba to psát všude

/***********************************************************************************************************************/

auto &man = rb::Manager::get(); // vytvoří referenci

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

    void getDockColor(int dock_index) {
        if (dock_index < 0 || dock_index >= DOCK_COUNT) {
            Serial.printf("Chyba: Neplatný index docku %d\n", dock_index);
            return;
        }
        const Dock& dock = docks[dock_index];
        Serial.printf("Dock %d má barvu: %s\n", dock_index, dock.colorToString().c_str());
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
typedef enum
    {
        BACK,
        RIGHT,
        LEFT,
        FRONT,
    } USid;

    int GetUS(USid ultrasound_Id)
    {
        auto &man = rb::Manager::get(); // get manager instance as singleton
        int dist = 0;
        if      (ultrasound_Id == BACK)     { dist = man.ultrasound(0).measure(); } 
        else if (ultrasound_Id == RIGHT)    { dist = man.ultrasound(1).measure(); } 
        else if (ultrasound_Id == LEFT)     { dist = man.ultrasound(2).measure(); } 
        else if (ultrasound_Id == FRONT)    { dist = man.ultrasound(3).measure(); } 

        if (dist == 0) { dist = 2000; } // treat 0 as max distance
        return dist;
    }

std::atomic<bool> IsEnemy(false);

// Funkce pro detekci soupeře pomocí ultrazvukové věže
void EnemyDetection() {
        while (true)
    {
        int distRight = 0, distLeft = 0, distFront = 0, distBack = 0;
        for (int i = 0; i < 3; ++i)
        {
            distRight += GetUS(USid::RIGHT);
            distLeft += GetUS(USid::LEFT);
            distFront += GetUS(USid::FRONT);
            distBack += GetUS(USid::BACK);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        distRight /= 3;
        distLeft /= 3;
        distFront /=3;
        distBack /= 3;


        //if (distRight < 250 || distLeft < 250)
        if(distBack < 250)
        {
            IsEnemy = true;
        }
        // else if (distRight < 500 || distLeft < 500)
        // {
        //     // Pokud je detekována vzdálenost menší než 500mm, ale větší než 250mm, považujeme to za potenciálního nepřítele
        //     // a nastavíme IsEnemy na true, ale nebudeme to brát jako jistotu
        //     IsEnemy = false; // nebo true, podle potřeby
        // }
        // else if (distRight > 1000 && distLeft > 1000 && distFront > 1000 && distBack > 1000)
        // {
        //     IsEnemy = true;
        // }
        // else
        // {
        //     IsEnemy = false;
        // }
        // Serial.printf("US Right AVG: %d, US Left AVG: %d, IsEnemy: %s\n", distRight, distLeft, IsEnemy.load() ? "true" : "false");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



/***********************************************************************************************************************/

//POHYBY

const float korekceM1 = 0.92f;
const float korekceM4 = 0.92f; // korekce pro M2, pokud je potřeba

int Odchylka = 0, Integral = 0, Last_odchylka = 0;
void zkontroluj_pid(int power, int M1_pos, int M4_pos){
    int Max_integral = 1000;
    Odchylka = M1_pos - M4_pos;
    Integral += Odchylka;
    if (Integral >  Max_integral) Integral =  Max_integral;
    if (Integral < -Max_integral) Integral = -Max_integral;

    int correction = Odchylka * Kp + Integral * (Ki+2) + (Odchylka - Last_odchylka) * Kd;

    int basePower = power;
    int leftPower = basePower - correction;
    int rightPower = basePower + correction;

    const int maxPower = 32000;
    const int minPower = 800;

    leftPower = constrain(leftPower, -maxPower, maxPower);
    rightPower = constrain(rightPower, -maxPower, maxPower);

    // Minimální výkon pro lepší dojezd
    if (abs(leftPower) < minPower && leftPower != 0) leftPower = (leftPower > 0) ? minPower : -minPower;
    if (abs(rightPower) < minPower && rightPower != 0) rightPower = (rightPower > 0) ? minPower : -minPower;

    auto& man = rb::Manager::get();

    man.motor(rb::MotorId::M1).power(-leftPower * korekceM1);
    man.motor(rb::MotorId::M4).power(rightPower * korekceM4);

    Last_odchylka = Odchylka;
}

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

            volatile int M1_pos = 0;
            volatile int M4_pos = 0;

            while (traveled < distance && !enemyDetected) {
                // Asynchronní čtení pozic motorů
                man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) { M1_pos = abs(info.position()); });
                man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) { M4_pos = abs(info.position()); });

                vTaskDelay(8 / portTICK_PERIOD_MS); // Dej callbackům čas

                float avgPos = (M1_pos + M4_pos) / 2.0f;

                //const float correction = 0.92f; // nebo 870.0/800.0
                traveled = avgPos * (67 * PI) / (21.5467f * 48.f) / korekceM1;

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
                if (IsEnemy) {
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

                // PID řízení motorů pomocí tvé funkce
                int power = static_cast<int>(currentSpeed * 32000 / 100);
                zkontroluj_pid(direction * power, M1_pos, M4_pos);

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
    struct AbsMoveTask {
        int targetPos;
        float maxSpeed;
        float accel;
        std::function<void(bool)> onFinish;
    };

    AbsMoveTask* taskParams = new AbsMoveTask{
        absTargetPos,
        speed,
        speed / 2.0f,
        callback
    };

    xTaskCreate(
        [](void* params) {
            AbsMoveTask* task = static_cast<AbsMoveTask*>(params);
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

                vTaskDelay(8 / portTICK_PERIOD_MS);

                float avgPos = (M1_pos + M4_pos) / 2.0f;
                traveled = avgPos * (67 * PI) / (21.5467f * 48.f) / korekceM1;

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
                if (IsEnemy) {
                    enemyDetected = true;
                    break;
                }

                // PID řízení motorů pomocí tvé funkce
                int power = static_cast<int>(currentSpeed * 32000 / 100);
                zkontroluj_pid(direction * power, M1_pos, M4_pos);

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
        taskParams,
        1,
        nullptr
    );
}

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

// Enum pro vlastní barvy tlačítek nebo bloků
enum class MyColor {
    RED,
    BLUE,
    NON
};

MyColor myColor;

class Robot{
private:
    MyColor myColor;

public:
    //Konstruktor
    Robot(MyColor c = MyColor::RED)
        : myColor(c) {}
    
    //Getery a settery
    MyColor getColor() const    { return myColor;}
    void setColor(MyColor c)    { myColor = c; }

    std::string colorToString() const {
        switch(myColor){
            case MyColor::NON: return "NON";
            case MyColor::RED: return "RED";
            case MyColor::BLUE: return "BLUE";
        }
    }

};

Robot robot;

/*****************************************************************************************************************************/


void WaitForStart(){
    while (true)
    {
        if (man.buttons().on() == 1)
        {
            break;
        }
        delay(10);
    }
}


void CheckBattery()
{
    const auto &bat = man.battery();
    static const uint32_t VOLTAGE_MAX = 8000; //%edit
    static const uint32_t VOLTAGE_MIN = 6000; //%edit
    int i = 0;
    int voltage = 0;
    for (i = 0; i < 2; ++i)
    {
        voltage = bat.voltageMv();
        // Vypočítej procenta (omez na 0-100)
        int pct = (voltage - VOLTAGE_MIN) * 100 / (VOLTAGE_MAX - VOLTAGE_MIN);
        if (pct > 100)
            pct = 100;
        if (pct < 0)
            pct = 0;

        if (i > 0)
        {
            printf("Battery at %d%%, %dmv\n", pct, voltage);
            if (voltage < VOLTAGE_MIN)
            {
                printf("Je třeba nabít baterii!\n");
            }
        }
        delay(1000);
    }
}


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
    
    rkLedRed(false); // Vypnutí červené LED
    rkLedBlue(false); 
    rkLedGreen(true);
    rkLedYellow(true);
    
    // delay(1000); // Počkáme 1 sekundu, aby se vše inicializovalo
    // moveToDockAsync(0, 60.0f, [](bool success, Color color) {});
    // delay(5000); // Počkáme 1 sekundu, aby se vše inicializovalo


    //moveToAbsolutePositionAsync(0, 60.0f, [](bool success) {});
    //move.Stop(); // Zastavení robota

    std::thread Ultrasonic(EnemyDetection); // Spustí detekci soupeře v samostatném vlákně

    while(true){

        //Nastaví LED podle barvy
        if(robot.getColor() == MyColor::RED){
            rkLedRed(true);
            rkLedBlue(false);
        } else if(robot.getColor() == MyColor::BLUE){
            rkLedRed(false);
            rkLedBlue(true);
        }

        //Toggle přepínání barvy robota
        if (rkButtonIsPressed(BTN_UP)){
            robot.setColor((robot.getColor() == MyColor::RED) ? MyColor::BLUE : MyColor::RED);
            delay(300);
        }

        if(rkButtonOn()) { break; }
        delay(200); 

        Serial.printf("Ultrazvuk: %d\n", GetUS(USid::BACK));

    }
        //Vypíše barvu za kterou jede robot
        Serial.printf("Naše barva je: %s\n", robot.colorToString().c_str());

        // //Přijede ke zdi
        // rkMotorsSetPower(30, 30);
        // delay(1000);
        // rkMotorsSetPower(0, 0);

        moveToAbsolutePositionAsync(1000, 60.0f, [](bool success) {});
        delay(8000);
        //moveToAbsolutePositionAsync(200, 60.0f, [](bool success) {});


        //dock.Color
        Ultrasonic.detach(); // Uvolní vlákno detekce soupeře
}
/*****************************************************************************************************************************/
/*****************************************************************************************************************************/

// Hlavní smyčka programu
void loop() {

    


        // //Left button BLUE // Nastavení barvy na modrou
        // if ((digitalRead(Bbutton1) == LOW)) { myColor = BLUE; }
    
        // //Right button RED // Nastavení barvy na červenou
        // if ((digitalRead(Bbutton2) == LOW)) { myColor = RED; }  


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