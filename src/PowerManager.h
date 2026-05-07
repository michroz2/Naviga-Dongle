/**
 * File: PowerManager.h
 * Version: 1.21 Изменение: Абстрагирование питания. Отключение AXP2101 для плат без него (Шаг 3.1).
 * Description: Изолированный класс для управления питанием.
 */
 #ifndef POWER_MANAGER_H
 #define POWER_MANAGER_H
 
 #include <Arduino.h>
 #include <Wire.h>
 #include "configuration.h"
 
 // Подключаем библиотеку PMU только если чип аппаратно присутствует (T-Beam)
 #if HAS_PMU
 #include <XPowersLib.h>
 #endif
 
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
 #if HAS_PMU
     XPowersAXP2101 _pmu; // Экземпляр драйвера питания
 #endif
 };
 
 #endif // POWER_MANAGER_H