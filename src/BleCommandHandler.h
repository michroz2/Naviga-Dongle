/**
 * Project: Naviga-Dongle
 * File: BleCommandHandler.h
 * Version: 1.43.7
 * Description: Диспетчер команд Bluetooth. Читает флаги от BleManager 
 * и раздает команды другим подсистемам (DB, Tx, Settings).
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
     
     // myNodeId передается как const, так как Оператор не имеет права его менять!
     const uint8_t& _myNodeId;
     
     // myNodeType можно менять (например, переключить со Stalker на Tracker)
     uint8_t& _myNodeType;
 };
 
 #endif // BLE_COMMAND_HANDLER_H