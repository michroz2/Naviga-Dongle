/**
 * Project: Naviga-Dongle
 * File: BleCommandHandler.h
 * Version: 1.46.6
 * Description: Диспетчер команд Bluetooth. Читает флаги от BleManager 
 * и раздает команды другим подсистемам (DB, Tx, Settings).
 * Изменение: Добавлен таймер для Rate-Limiting выгрузки базы (Fix 1.46.6).
 */

 #ifndef BLE_COMMAND_HANDLER_H
 #define BLE_COMMAND_HANDLER_H
 
 #include <Arduino.h>
 #include "BleManager.h"
 #include "DBManager.h"
 #include "SettingsManager.h"
 #include "TxManager.h"
 #include "NodeDatabase.h"
 #include "logger.h"
 
 class BleCommandHandler {
 public:
     BleCommandHandler(BleManager& ble, DBManager& db, SettingsManager& settings,
                       TxManager& tx, NodeDatabase& nodeDB, 
                       const uint8_t& myNodeId, uint8_t& myNodeType);
 
     // Основной метод проверки флагов, вызывается в loop()
     void process();
 
 private:
     BleManager& _ble;
     DBManager& _db;
     SettingsManager& _settings;
     TxManager& _tx;
     NodeDatabase& _nodeDB;
     
     const uint8_t& _myNodeId;
     uint8_t& _myNodeType;
 
     // Переменная состояния для асинхронной выгрузки базы по Bluetooth
     // 0 - выгрузка не идет. 1..254 - ID узла, с которого нужно продолжить проверку.
     uint8_t _syncBookmark; 
     
     // НОВОЕ 1.46.6: Защита от переполнения BLE буфера
     uint32_t _lastSyncSendTime; 
     static const uint32_t BLE_SYNC_PACKET_DELAY_MS = 20; // Окно безопасной передачи (20 мс)
 };
 
 #endif // BLE_COMMAND_HANDLER_H