/**
 * File: PowerManager.h
 * Version: 1.0.0
 * Description: Изолированный класс для управления чипом питания AXP2101.
 */
 #ifndef POWER_MANAGER_H
 #define POWER_MANAGER_H
 
 #include <Arduino.h>
 #include <Wire.h>
 #include <XPowersLib.h>
 #include "configuration.h"
 
 class PowerManager {
 public:
     PowerManager();
 
     // Инициализация чипа и включение нужных линий питания
     bool init();
 
     // Сброс питания на линии ALDO3 (используется для спасения зависшего GPS)
     void cycleGpsPower();
 
     // Задел на будущее: получение уровня заряда батареи
     // int getBatteryLevel();
 
 private:
     XPowersAXP2101 _pmu;
 };
 
 #endif // POWER_MANAGER_H