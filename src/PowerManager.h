/**
 * File: PowerManager.h
 * Version: 1.34 
 * Изменение: Добавлены методы чтения напряжения и процента заряда батареи для отправки телеметрии.
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
 
     // Получение телеметрии батареи (Реализовано в v1.34)
     uint8_t getBatteryPercent();
     uint16_t getBatteryVoltage();
 
 private:
 #if HAS_PMU
     XPowersAXP2101 _pmu; // Экземпляр драйвера питания
 #endif
 };
 
 #endif // POWER_MANAGER_H