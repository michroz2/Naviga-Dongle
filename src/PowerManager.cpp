/**
 * File: PowerManager.cpp
 * Version: 1.34 
 * Изменение: Реализована логика измерения заряда для плат с AXP2101 и прямой расчет по АЦП для T-Energy S3.
 * Description: Реализация класса управления питанием.
 */
 #include "PowerManager.h"
 #include "logger.h"
 
 PowerManager::PowerManager() {
     // Конструктор пока пуст, вся работа в init()
 }
 
 bool PowerManager::init() {
 #if HAS_PMU
     // AXP2101 сидит на той же шине I2C, что и дисплей (T-Beam v1.1)
     if (_pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
         LOG_INFO("PWR", "AXP2101 PMU initialized successfully.");
         
         // Включаем питание LoRa (LDO линия ALDO2)
         _pmu.setALDO2Voltage(3300); 
         _pmu.enableALDO2();
         
         // Включаем питание GPS (LDO линия ALDO3)
         _pmu.setALDO3Voltage(3300); 
         _pmu.enableALDO3();
         
         // Отключаем неиспользуемые линии для экономии
         _pmu.disableALDO4(); 
         
         // Включаем АЦП для замера батареи 
         _pmu.enableSystemVoltageMeasure();  
         return true;
     }
     LOG_WARN("PWR", "AXP2101 PMU initialization FAILED!");
     return false;
 #else
     // Если на плате нет контроллера питания (как на T-Energy S3), 
     // просто имитируем успешную инициализацию (Dummy mode).
     LOG_INFO("PWR", "No PMU on this board. Direct power is used.");
     return true; 
 #endif
 }
 
 // Жесткий аппаратный сброс питания GPS-модуля (Используется для вывода Ublox из зависших состояний)
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
     // Оставляем только логирование. Встроенный программный Rescue Mode (через UBX_FACTORY_RESET) в GpsManager отработает сам.
     LOG_WARN("PWR", "Cannot cycle GPS power: No PMU hardware.");
 #endif
 }
 
 // Получение процента заряда батареи
 uint8_t PowerManager::getBatteryPercent() {
 #if HAS_PMU
     return _pmu.getBatteryPercent();
 #else
     // Для плат без PMU (T-Energy S3) вычисляем процент по напряжению.
     // Полностью заряженный Li-Ion ~ 4.2V (4200 mV), полностью разряженный ~ 3.3V (3300 mV).
     uint16_t mv = getBatteryVoltage();
     if (mv >= 4200) return 100;
     if (mv <= 3300) return 0;
     return (uint8_t)map(mv, 3300, 4200, 0, 100);
 #endif
 }
 
 // Получение напряжения батареи в милливольтах
 uint16_t PowerManager::getBatteryVoltage() {
 #if HAS_PMU
     return _pmu.getBattVoltage(); // Возвращает милливольты через драйвер
 #else
     // Для плат T-Energy S3: аппаратный делитель напряжения 1/2 подключен к GPIO 3.
     // analogReadMilliVolts возвращает напряжение на самом пине. Умножаем на 2.
     uint32_t adc_mv = analogReadMilliVolts(3);
     return (uint16_t)(adc_mv * 2);
 #endif
 }
 //PowerManager.cpp