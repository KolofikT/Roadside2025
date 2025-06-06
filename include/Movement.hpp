#include "Arduino.h"
#include "robotka.h"
#include <iostream>
#include <thread>
#include "RBCX.h"
//#include "smart_servo_command.h"

// Funkce pro detekci soupeře pomocí ultrazvukové věže
bool EnemyDetection() {
    return false;
}



struct Movement
{
  rb::MotorId motorL = rb::MotorId::M1; // levý motor
  rb::MotorId motorR = rb::MotorId::M4; // pravý motor
  double mm_to_ticks = 0.215;           // konstanta pro prepocet tics enkoderu na mm
  int wheel_base = 165;                 // vzdalenost mezi koly robota v mm
  int last_ticks_M3 = 0;                // pravy motor
  int last_ticks_M2 = 0;                // levy motor

  /**
   * Fuknkce pro pohyb robota v oblouku doprava
   * @param angle: jaky uhel oblouku robot ujede
   * @param radius: polomer oblouku vnitrniho kola robota
   *
   * @return None
   */
  void ArcRight(int angle, int radius)
  {
    auto &man = rb::Manager::get(); // vytvoří referenci
    // Nastaveni pocitadla tics na 0 pro oba motory
    man.motor(motorL).setCurrentPosition(0);
    man.motor(motorR).setCurrentPosition(0);
    // auto &man = rb::Manager::get();
    double inner_lenght = (((2 * PI * radius) / 360) * angle) / mm_to_ticks;                  // vzdalenost, kterou musi vnitrni kolo v oblouku ujet v tics enkoderu
    double outer_lenght = (((2 * 3.14 * (radius + wheel_base)) / 360) * angle) / mm_to_ticks; // vzdalenost, kterou musi vnejsi kolo v oblouku ujet v tics enkoderu
    // Serial.println(inner_lenght);
    int outer_sped = 3200;                                         // rychlost vnejsiho kola
    int inner_speed = -(outer_sped * inner_lenght) / outer_lenght; // vypocet pro rychlost vnitrniho kola
    // Serial.println(inner_speed);
    int tics_M2 = 0;
    int tics_M3 = 0;
    // Serial.println(outer_lenght);
    while ((inner_lenght > tics_M3) && (outer_lenght > tics_M2))
    {
      man.motor(motorL).speed(-outer_sped);
      man.motor(motorR).speed(-inner_speed);
      man.motor(motorL).requestInfo([&tics_M2](rb::Motor &info)
                                    { tics_M2 = info.position(); });
      man.motor(motorR).requestInfo([&tics_M3](rb::Motor &info)
                                    { tics_M3 = info.position(); });
      delay(10);
    }
  }

  void Arcleft(int angle, int radius)
  {
    auto &man = rb::Manager::get(); // vytvoří referenci
    man.motor(motorL).setCurrentPosition(0);
    man.motor(motorR).setCurrentPosition(0);
    //auto &man = rb::Manager::get();
    double inner_lenght = (((2 * PI * radius) / 360) * angle) / mm_to_ticks;
    double outer_lenght = (((2 * 3.14 * (radius + wheel_base)) / 360) * angle) / mm_to_ticks;
    Serial.println(inner_lenght);
    int outer_sped = -3200;
    int inner_speed = -(outer_sped * inner_lenght) / outer_lenght;
    Serial.println(inner_speed);
    int tics_M2 = 0;
    int tics_M3 = 0;
    Serial.println(outer_lenght);
    while ((inner_lenght > tics_M2) && (outer_lenght > tics_M3))
    {
      man.motor(motorR).speed(-outer_sped);
      man.motor(motorL).speed(-inner_speed);
      man.motor(motorR).requestInfo([&tics_M3](rb::Motor &info)
                                    {
            //printf("M3: position:%d\n", info.position());
            tics_M3 = info.position(); });
      man.motor(motorL).requestInfo([&tics_M2](rb::Motor &info)
                                    { tics_M2 = info.position(); });
      delay(10);
    }
  }

  void Acceleration(int speed_from, int speed_to, int distance_mm)
  {
    auto &man = rb::Manager::get(); // vytvoří referenci
    man.motor(motorL).setCurrentPosition(0);
    man.motor(motorR).setCurrentPosition(0);
    double distance_ticks = distance_mm / mm_to_ticks;
    double acc_const = speed_to / distance_ticks;
    int ticks_M2 = 0;
    int ticks_M3 = 0;
    int error = 0;
    while ((ticks_M2 < distance_ticks) && (ticks_M3 < distance_ticks))
    {
      error = 10 * (ticks_M2 - ticks_M3);
      man.motor(motorL).speed(-(acc_const * ticks_M2 + speed_from - error));
      man.motor(motorR).speed(acc_const * ticks_M3 + speed_from + error);
      man.motor(motorR).requestInfo([&ticks_M3](rb::Motor &info)
                                    {
            //printf("M3: position:%d\n", info.position());
            ticks_M3 = info.position(); });
      man.motor(motorL).requestInfo([&ticks_M2](rb::Motor &info)
                                    {
            //printf("M2: position:%d\n", info.position());
            ticks_M2 = -info.position(); });

      delay(10);
    }
  }

  void TurnRight(int angle)
  {
    auto &man = rb::Manager::get(); // vytvoří referenci
    man.motor(motorL).setCurrentPosition(0);
    man.motor(motorR).setCurrentPosition(0);
    int ticks_M2 = 0;
    int ticks_M3 = 0;
    int distance = ((PI * wheel_base) / 360) * angle / mm_to_ticks;
    while (distance > ticks_M3)
    {
      man.motor(motorL).speed(500);
      man.motor(motorR).speed(500);
      man.motor(motorR).requestInfo([&ticks_M3](rb::Motor &info)
                                    {
            //printf("M3: position:%d\n", info.position());
            ticks_M3 = info.position(); });
      man.motor(motorL).requestInfo([&ticks_M2](rb::Motor &info)
                                    {
            //printf("M2: position:%d\n", info.position());
            ticks_M2 = -info.position(); });

      delay(10);
    }
    man.motor(motorR).speed(0);
    man.motor(motorL).speed(0);
  }

  void TurnLeft(int angle)
  {
    auto &man = rb::Manager::get(); // vytvoří referenci
    man.motor(motorL).setCurrentPosition(0);
    man.motor(motorR).setCurrentPosition(0);
    int ticks_M2 = 0;
    int ticks_M3 = 0;
    int distance = ((PI * wheel_base) / 360) * angle / mm_to_ticks;
    while (distance > ticks_M3)
    {
      man.motor(motorL).speed(-500);
      man.motor(motorR).speed(-500);
      man.motor(motorR).requestInfo([&ticks_M3](rb::Motor &info)
                                    {
            //Serial.println( -info.position());//"M3: position:%d\n",
            ticks_M3 = -info.position(); });
      man.motor(motorL).requestInfo([&ticks_M2](rb::Motor &info)
                                    {
            //printf("M2: position:%d\n", info.position());
            ticks_M2 = info.position(); });

      delay(10);
    }
    man.motor(motorR).speed(0);
    man.motor(motorL).speed(0);
  }

  void BackwardUntillWall()
  {
    auto &man = rb::Manager::get(); // vytvoří referenci
    const int ultrasound_Id = 0;  // us id
    const int stop_distance = 50; // vzdálenost v mm, kdy má robot zastavit

    // Funkce pro průměr ze tří měření
    auto averageDistance = [&]() -> int
    {
      int d1 = man.ultrasound(ultrasound_Id).measure();
      delay(5);
      int d2 = man.ultrasound(ultrasound_Id).measure();
      delay(5);
      int d3 = man.ultrasound(ultrasound_Id).measure();
      return (d1 + d2 + d3) / 3;
    };

    int distance = averageDistance();
    delay(1);

    while (distance > stop_distance) // dokud není překážka blízko
    {
      man.motor(motorL).speed(2500);
      man.motor(motorR).speed(-2500);

      distance = averageDistance();
      // printf("Distance: %4d mm (%3d cm)\n", distance, distance / 10);
    }
    delay(2000);

    man.motor(motorL).speed(0);
    man.motor(motorR).speed(0);
  }

  /**
   * Funkce pro pohyb robota rovne
   * @param speed: rychlost kterou robot pojede
   * @param distance: vzdalenost kterou ma robot ujet v mm
   * @param timeout: po jake dobe se ma funkce ukoncit i kdyz neujede pozadovanou vzdalenost
   * @return None
   */
  void Straight( int distance, int speed, int timeout, int correction)
  {
    auto &man = rb::Manager::get(); // vytvoří referenci
    man.motor(motorL).setCurrentPosition(0);
    man.motor(motorR).setCurrentPosition(0);
    int time = 0;
    int ticks_M2 = 0;
    int ticks_M3 = 0;
    int distance_ticks = distance / mm_to_ticks;

    while (ticks_M2 < distance_ticks && time < timeout)
    {
      if (EnemyDetection()) // pokud je detekován nepřítel
      {
        // Stop motors
        man.motor(motorL).speed(0);
        man.motor(motorR).speed(0);
        // Serial.println("Enemy detected! Waiting...");
      }
      else
      {
        man.motor(motorL).speed(-speed+ correction);
        man.motor(motorR).speed(speed- correction);
        man.motor(motorR).requestInfo([&ticks_M3](rb::Motor &info)
                                      { ticks_M3 = info.position(); });
        man.motor(motorL).requestInfo([&ticks_M2](rb::Motor &info)
                                      { ticks_M2 = -info.position(); });
        //Serial.printf("M2: %d, M3: %d\n", ticks_M2, ticks_M3);
        delay(10);
        time = time + 10;
      }
    }
    // Stop at the end
    man.motor(motorL).speed(0);
    man.motor(motorR).speed(0);
  }

  void Stop()
  {
    auto &man = rb::Manager::get(); // vytvoří referenci
    man.motor(motorL).speed(0);
    man.motor(motorR).speed(0);
  }
};
