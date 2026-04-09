/**
 * File: PowerManager.cpp
 * Version: 1.21 Изменение: Изоляция вызовов AXP2101 для совместимости с T-Energy S3 (Шаг 3.1).
 * Description: Реализация класса управления питанием.
 */
 #include "PowerManager.h"
 #include "logger.h"
 
 PowerManager::PowerManager() {
     // Конструктор пока пуст, вся работа в init()
 }
 
 bool PowerManager::init() {
 #if HAS_PMU
     // AXP2101 сидит на той же шине I2C, что и дисплей
     if (_pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
         LOG_INFO("PWR", "AXP2101 PMU initialized successfully.");
         
         // Включаем питание LoRa (ALDO2)
         _pmu.setALDO2Voltage(3300); 
         _pmu.enableALDO2();
         
         // Включаем питание GPS (ALDO3)
         _pmu.setALDO3Voltage(3300); 
         _pmu.enableALDO3();
         
         // Отключаем неиспользуемое
         _pmu.disableALDO4(); 
         
         // Включаем АЦП для замера батареи в будущем
         _pmu.enableSystemVoltageMeasure();  
         return true;
     }
     LOG_WARN("PWR", "AXP2101 PMU initialization FAILED!");
     return false;
 #else
     // Если на плате нет контроллера питания (как на T-Energy S3), 
     // просто имитируем успешную инициализацию.
     LOG_INFO("PWR", "No PMU on this board. Direct power is used.");
     return true; 
 #endif
 }
 
 void PowerManager::cycleGpsPower() {
 #if HAS_PMU
     LOG_INFO("PWR", "Cycling GPS power (ALDO3)...");
     _pmu.disableALDO3(); 
     delay(2000); 
     _pmu.enableALDO3(); 
     delay(2000); 
     LOG_INFO("PWR", "GPS power restored.");
 #else
     // Если PMU нет, мы не можем аппаратно передернуть питание GPS.
     // Оставляем только логирование. Встроенный программный Rescue Mode в GpsManager отработает сам.
     LOG_WARN("PWR", "Cannot cycle GPS power: No PMU hardware.");
 #endif
 }