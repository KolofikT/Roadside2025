//RBCX - Robotka
#include <Arduino.h>
#include "robotka.h"
#include "smart_servo_command.h"
#include "motor_commands.h"

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
    void initialize(int start_position = 0) {
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


// Upravená funkce pro pohyb dopředu s aktualizací pozice robota
void move_straight_with_tracking(float mm, float speed) {
    forward(mm, speed); // původní funkce pro pohyb
    
    if(speed < 0) mm *= -1;  // pokud je rychlost záporná, invertujeme vzdálenost

    // Aktualizace pozice - pohyb dopředu přičítá vzdálenost
    positionTracker.updatePosition(static_cast<int>(mm));
    
    Serial.printf("Pohyb dopredu o %.1f mm. Aktualni pozice: %d mm\n", 
                 mm, positionTracker.getCurrentPosition());
}

// Funkce pro získání aktuální pozice
int getRobotPosition() {
    return positionTracker.getCurrentPosition();
}

void navigateToDock(int dockIndex) {
    try {
        // Získání pozice docku
        int dockPos = manager.getDockAbsolutePosition(dockIndex);
        
        // Získání aktuální pozice robota
        int robotPos = positionTracker.getCurrentPosition();
        
        // Výpočet potřebné vzdálenosti
        int distance = dockPos - robotPos;
        
        if (distance > 0) {
            move_straight_with_tracking(static_cast<float>(distance), 60.0f); // Pohyb vpřed k docku
        } else {
            Serial.println("Jsme u cílového docku!");
        }
        
        //Naložení baterie do docku
        //load_dock();

        // Označení docku jako obsazený
        manager.getDock(dockIndex).setStatus(Status::FILLED);
        
    } catch (const std::out_of_range& e) {
        Serial.println("Chyba: Neplatný index docku!");
    }
}

/*****************************************************************************************************************************/

//OVLÁDÍNÍ RAMENE

    // Funkce pro naložení baterie na rameno
void load_battery() {
    auto &bus = rkSmartServoBus(2);
    s_s_move(bus, 0, 50, 80.0);     // Rameno - nahoře
    rkServosSetPosition(1, 90);     // Magnet servo drží
    s_s_move(bus, 1, 215, 80.0);    // Rameno - levá
    delay(5000);
    s_s_move(bus, 0, 120, 80.0);    // Rameno - dole
    delay(5000);
    s_s_move(bus, 0, 60, 80.0);     // Rameno - nahoře - aktiv
    delay(5000);
}

    // Funkce pro vyložení baterie z ramene
void unload_battery(int dockIndex) {
    auto &bus = rkSmartServoBus(2);
    s_s_move(bus, 1, 35, 80.0);     // Rameno - pravá
    delay(5000);
    s_s_move(bus, 0, 120, 80.0);    // Rameno - dole
    delay(5000);
    rkServosSetPosition(1, 0);      // Magnet servo pouští
    delay(5000);
    s_s_move(bus, 0, 50, 80.0);     // Rameno - nahoře
    delay(5000); 
    rkServosSetPosition(1, 90);     // Magnet servo drží
    s_s_move(bus, 1, 125, 80.0);    // Rameno - střed
    
    // Označení docku jako obsazený
    manager.getDock(dockIndex).setStatus(Status::FILLED);
}

    // Funkce pro naložení baterie a rovnou ji přesunout do docku
void load_dock(int dockIndex) {
    auto &bus = rkSmartServoBus(2);
    s_s_move(bus, 0, 50, 80.0);     // Rameno - nahoře
    rkServosSetPosition(1, 90);     // Magnet servo drží
    s_s_move(bus, 1, 215, 80.0);    // Rameno - levá
    delay(5000);
    s_s_move(bus, 0, 120, 80.0);    // Rameno - dole
    delay(5000);
    s_s_move(bus, 0, 60, 80.0);     // Rameno - nahoře - aktiv
    delay(5000);
    s_s_move(bus, 1, 35, 80.0);     // Rameno - pravá
    delay(5000);
    s_s_move(bus, 0, 120, 80.0);    // Rameno - dole
    delay(5000);
    rkServosSetPosition(1, 0);      // Magnet servo pouští
    delay(5000);
    s_s_move(bus, 0, 50, 80.0);     // Rameno - nahoře
    delay(5000); 
    rkServosSetPosition(1, 90);     // Magnet servo drží
    s_s_move(bus, 1, 125, 80.0);    // Rameno - střed
    // Označení docku jako obsazený
    manager.getDock(dockIndex).setStatus(Status::FILLED);
}

/*****************************************************************************************************************************/
/*****************************************************************************************************************************/
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

    printf("Robotka started!\n");
    
    // Nastavení pinů pro tlačítka
    pinMode(Bbutton1, INPUT_PULLUP);
    pinMode(Bbutton2, INPUT_PULLUP);
    
    //Nastavení smart servos
    auto &bus = rkSmartServoBus(2);
    //auto &bus = rkSmartServoBus(1);
    s_s_init(bus, 1, 30, 220);
    s_s_init(bus, 0, 40, 125);

    rkLedRed(true); 
    rkLedBlue(true); 

    // Inicializace sledování pozice
    positionTracker.initialize(0); // Startovní pozice 0 mm
    
    // Nastavení absolutních pozic pro docks:
    manager.getDock(0).setAbsolutePos(800);
    manager.getDock(1).setAbsolutePos(995);
    manager.getDock(2).setAbsolutePos(1190);
    manager.getDock(3).setAbsolutePos(1385);
    manager.getDock(4).setAbsolutePos(1580);
    manager.getDock(5).setAbsolutePos(1775);
    manager.getDock(6).setAbsolutePos(1970);
    manager.getDock(7).setAbsolutePos(2165);



    //    Hledání nejbližšího prázdného docku k pozici 250
    // int nearest_empty = manager.findNearestEmptyDock(250);
    
    // if (nearest_empty != -1) {
    //     std::cout << "Nejblizsi prazdny dock je cislo " << nearest_empty 
    //               << " na pozici " << manager.getDock(nearest_empty).getAbsolutePos() 
    //               << " mm" << std::endl;
    // } else {
    //     std::cout << "Neni k dispozici zadny prazdny dock" << std::endl;
    // }


}

/*****************************************************************************************************************************/
/*****************************************************************************************************************************/

// Hlavní smyčka programu
void loop() {

  auto &bus = rkSmartServoBus(2);
  //auto &bus = rkSmartServoBus(1);

  if (rkButtonIsPressed(BTN_UP)) {
        load_dock(0);
  }
  if (rkButtonIsPressed(BTN_DOWN)) {
        s_s_move(bus, 0, 120, 80.0);    // Rameno - dole
    }
  if (rkButtonIsPressed(BTN_ON)) {
        move_straight_with_tracking(200, 40);   // pohyb vpřed s trackingem
        move_straight_with_tracking(100, -40);  // pohyb vzad s trackingem
        getRobotPosition(); // Získání aktuální pozice robota
        printf("Aktuální pozice robota: %d mm\n", getRobotPosition());
  }
  if (rkButtonIsPressed(BTN_OFF)) {
        navigateToDock(0);
  }
  if ((digitalRead(Bbutton1) == LOW)){
        rkLedYellow(true); // Turn on red LED
        rkLedGreen(false); // Turn on red LED

        s_s_move(bus, 1, 215, 80.0);    // Rameno - levá
    }
  if ((digitalRead(Bbutton2) == LOW)){
        rkLedGreen(true); // Turn on red LED
        rkLedYellow(false); // Turn on red LED

        s_s_move(bus, 1, 35, 80.0);     // Rameno - pravá
    }
    // Pro jistotu, aby se cyklus neprováděl příliš rychle
  delay(50);
}