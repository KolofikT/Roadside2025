//RBCX - Robotka
#include <Arduino.h>
#include "robotka.h"
#include "smart_servo_command.h"
#include "motor_commands.h"
#include "RBCXMotor.h"

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
// // PID řízení pro pohyb robota	
// float M_wheel_circumference = 127.0f * PI; // Průměr kola v mm * PI
// int32_t MmToTicks(float mm){
//     return (mm / M_wheel_circumference) * 38.55937f * 48.f;
// }

// float TicksToMm(int32_t ticks) {
//     return float(ticks) / 38.55937f / 48.f * M_wheel_circumference;
// }

// int Odchylka = 0, Integral = 0, Last_odchylka = 0;
// void zkontroluj_pid(int power, int M1_pos, int M4_pos){
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
// ///////////////////
// int M4_pos = 0;
// int M1_pos = 0;

// void jed_a_chytej_puky(int distance, bool x, bool kladna = true){
//   auto& man = rb::Manager::get(); // vytvoří referenci na man class
//   man.motor(rb::MotorId::M1).setCurrentPosition(0);
//   man.motor(rb::MotorId::M4).setCurrentPosition(0);
//   otevreni_prepazky(); // Otevře prepážku
//   man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
//           M4_pos = info.position();
//       });
//   man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
//          M1_pos = -info.position();
//       });

//   xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);

//   Serial.printf("musi_jet::: M1_pos: %d, M4_pos: %d\n", MmToTicks(distance), MmToTicks(distance));
//   setMotorsPower(20000, 20000); // Nastaví motory na 20000

//   while(M4_pos < MmToTicks(distance) && M1_pos < MmToTicks(distance)) {
//           man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
//               M4_pos = info.position();
//           });
//           man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
//              M1_pos = -info.position();
//           });

//           Serial.printf("[ENCODERY] M4_pos: %d\n", M4_pos);

//           if(M1_pos > 200 && M4_pos > 200) {
//             zkontroluj_pid(20000, M1_pos, M4_pos);
//           }

//           if(detekce_soupere()) {
//            break;
//           }
//           delay(10);
//     }
//     man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
//               M4_pos = info.position();
//           });
//     man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
//              M1_pos = -info.position();
//           });
//     Serial.printf("[ENCODERY] M4_pos: %d, M1_pos: %d\n", M4_pos, M1_pos);
//     if(kladna){
//       if(x) {
//         aktualni_pozice.x = aktualni_pozice.x + TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
//       }
//       else{
//         aktualni_pozice.y = aktualni_pozice.y + TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
//       }
//     }
//     else{
//       if(x) {
//         aktualni_pozice.x = aktualni_pozice.x - TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
//       }
//       else{
//         aktualni_pozice.y = aktualni_pozice.y - TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
//       }
//     }
//       vTaskDelete(chytejPukyHandle);
//       chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
//       setMotorsPower(0, 0); // Zastaví motory
//       delay(100);
//       man.motor(rb::MotorId::M1).setCurrentPosition(0);
//       man.motor(rb::MotorId::M4).setCurrentPosition(0);
//       M1_pos = 0;
//       M4_pos = 0;
// }
/***********************************************************************************************************************/

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
            // Robot k docku jede dopředu
            move_straight_with_tracking(static_cast<float>(abs(distance)), 40.0f); // Pohyb vpřed k docku
        } else if (distance < 0) {
            //Robot k docku couvá
            move_straight_with_tracking(static_cast<float>(abs(distance)), -40.0f); // Pohyb vzad k docku
        } else {
            // Robot je již na správné pozici
            Serial.println("Jsme u cílového docku!");
            rkLedGreen(true); // Zapnutí zelené LED
        }
        
        //Naložení baterie do docku
        //load_dock();

        // Označení docku jako obsazený
        manager.getDock(dockIndex).setStatus(Status::FILLED);
        
    } catch (const std::out_of_range& e) {
        Serial.println("Chyba: Neplatný index docku!");
    }
}

void goToAbsolutePosition(int absPos) {

    // Získání aktuální pozice robota
    int robotPos = positionTracker.getCurrentPosition();
            
    // Výpočet potřebné vzdálenosti
    int distance = absPos - robotPos;
            
    if (distance > 0) {
        // Robot k docku jede dopředu
        move_straight_with_tracking(static_cast<float>(abs(distance)), 60.0f); // Pohyb vpřed k docku
    } else if (distance < 0) {
        //Robot k docku couvá
        move_straight_with_tracking(static_cast<float>(abs(distance)), -60.0f); // Pohyb vzad k docku
    } else {
        // Robot je již na správné pozici
        Serial.println("Jsme na cílové pozici!");
        rkLedGreen(true); // Zapnutí zelené LED
    }
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

    void Down(float speed = 80)     { auto &bus = rkSmartServoBus(2); s_s_move(bus, 0, 120, speed); }       // Rameno - dole

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
    }

    // Funkce pro vyložení baterie z ramene
    void unload_battery(int dockIndex, int WaiterIndex, float speed = 80) {
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

};

// Globální instance RamenoController
RamenoController Rameno;

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
    s_s_init(bus, 1, 30, 220); // Servo 1   Min 30     Max 220
    s_s_init(bus, 0, 40, 125); // Servo 0   Min 40     Max 125

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
  int pos = 50;
/*****************************************************************************************************************************/
/*****************************************************************************************************************************/

// Hlavní smyčka programu
void loop() {
    auto &bus = rkSmartServoBus(2);
  if (rkButtonIsPressed(BTN_UP)) {
    
    s_s_move(bus, 0, pos, 10);
    printf("Servo je na pozici %d\n", pos);//80
    pos++;

  }
  if (rkButtonIsPressed(BTN_DOWN)) {
        // move_straight_with_tracking(200, 40);
        // goToAbsolutePosition(0);
        printf("Kouká se\n");
        pixy.ccc.getBlocks();               // Získání bloků z Pixy2 kamery
        printf("Počet Blocků %d inicializováno\n", pixy.ccc.numBlocks);

        printf("Počet bloků: %d\n", pixy.ccc.numBlocks); // Výpis počtu detekovaných bloků

        if(pixy.ccc.blocks[0].m_signature == 1) { // Pokud je blok červený
            printf("Červený blok detekován!\n");
            // Zde můžete přidat další logiku pro zpracování červeného bloku
        } else if(pixy.ccc.blocks[0].m_signature == 2) { // Pokud je blok modrý
            printf("Modrý blok detekován!\n");
            // Zde můžete přidat další logiku pro zpracování modrého bloku
        } else {
            printf("Blok s jinou barvou detekován!\n");
        }



        // pixy.ccc.blocks[i].m_signature;     // Barva bloku (1 = červená, 2 = modrá, atd.)

        // Procházení detekovaných bloků a jejich výpis
        // if (pixy.ccc.numBlocks > 0) {
        //     for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        //         Serial.printf("Block %d: X=%d, Y=%d, Width=%d, Height=%d, Signature=%d\n", 
        //                       i, pixy.ccc.blocks[i].m_x, pixy.ccc.blocks[i].m_y, 
        //                       pixy.ccc.blocks[i].m_width, pixy.ccc.blocks[i].m_height, 
        //                       pixy.ccc.blocks[i].m_signature);
        //     }
        // } else {
        //     Serial.println("Žádné bloky nebyly detekovány.");
        // }
    }
  if (rkButtonIsPressed(BTN_ON)) {
    // move_straight_with_tracking(1000, 40);  // pohyb vpřed s trackingem
    //    Rameno.load_battery(3000);
  }
  if (rkButtonIsPressed(BTN_OFF)) {
        //positionTracker.initialize(); // Inicializace pozice robota na 70 mm
        // navigateToDock(0);
        // Rameno.load_dock(0, 3000); // Naložení baterie na rameno
        // navigateToDock(3);
        // navigateToDock(1);
        // navigateToDock(1);
        Rameno.unload_battery(1, 3000); // Vyložení baterie z ramene do docku 1

  }
  if ((digitalRead(Bbutton1) == LOW)){ //Left button
        rkLedYellow(true); // Turn on red LED
        rkLedGreen(false); // Turn on red LED

        Rameno.Left();
    }
  if ((digitalRead(Bbutton2) == LOW)){ //Right button 
        rkLedGreen(true); // Turn on red LED
        rkLedYellow(false); // Turn on red LED

        Rameno.Right();
    }
if(rkButtonIsPressed(BTN_LEFT)) {
        pixy.ccc.getBlocks();               // Získání bloků z Pixy2 kamery

        Serial.print("Number of blocks: ");
        Serial.println(pixy.ccc.numBlocks); 
    }
    // Pro jistotu, aby se cyklus neprováděl příliš rychle
  delay(50);
}