/**
 * Project: Naviga-Dongle
 * File: SmartPositionManager.h
 * Version: 1.00
 * Description: Класс управления адаптивной отправкой координат (Smart TX). 
 * Инкапсулирует логику принятия решений на основе скорости и смещения.
 */

 #ifndef SMART_POSITION_MANAGER_H
 #define SMART_POSITION_MANAGER_H
 
 #include <Arduino.h>
 #include "configuration.h"
 #include "logger.h"
 #include "GpsManager.h"
 #include "NodeDatabase.h"
 #include "TxManager.h"
 #include "BleManager.h"
 #include "SettingsManager.h"
 
 class SmartPositionManager {
 private:
     // Ссылки на внешние подсистемы (Dependency Injection)
     GpsManager& gps;
     NodeDatabase& nodeDB;
     TxManager& txManager;
     BleManager& bleManager;
     SettingsManager& settingsManager;
     
     // Ссылки на глобальные переменные для отслеживания изменений в реальном времени
     const uint8_t& myNodeId;
     const uint8_t& myNodeType;
     
     // Внутренний таймер последней успешной передачи
     uint32_t lastTxTime;
 
 public:
     /**
      * Конструктор менеджера позиции
      * Принимает ссылки на все необходимые компоненты оркестратора
      */
     SmartPositionManager(GpsManager& _gps, NodeDatabase& _nodeDB, TxManager& _txManager, 
                          BleManager& _bleManager, SettingsManager& _settingsManager,
                          const uint8_t& _myNodeId, const uint8_t& _myNodeType);
 
     /**
      * Сброс/Инициализация таймера передачи (вызывается в setup или при смене конфига)
      */
     void resetTimer();
 
     /**
      * Основной рабочий цикл менеджера. 
      * Анализирует GPS данные, считает дистанцию от последней точки и принимает решение об отправке.
      */
     void process();
 };
 
 #endif // SMART_POSITION_MANAGER_H