/**
 * File: PowerManager.cpp
 * Version: 1.0.0
 * Description: Реализация класса управления питанием.
 */
 #include "PowerManager.h"
 #include "logger.h"
 
 PowerManager::PowerManager() {
     // Конструктор пока пуст, вся работа в init()
 }
 
 bool PowerManager::init() {
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
 }
 
 void PowerManager::cycleGpsPower() {
     LOG_INFO("PWR", "Cycling GPS power (ALDO3)...");
     _pmu.disableALDO3(); 
     delay(2000); 
     _pmu.enableALDO3(); 
     delay(2000); 
     LOG_INFO("PWR", "GPS power restored.");
 }