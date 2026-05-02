/**
 * File: BleManager.h
 * Version: 1.29
 * Description: Управление BLE-стеком (NimBLE) и реализация протокола Naviga (UC-04).
 * Использует гибридную модель (Callbacks -> Флаги -> Основной цикл).
 */

 #ifndef BLE_MANAGER_H
 #define BLE_MANAGER_H
 
 #include <Arduino.h>
 #include <NimBLEDevice.h>
 #include "BleConfig.h"
 #include "BleProtocol.h"
 #include "DisplayManager.h" // Для enum BleStatus
 
 class BleManager {
 public:
     BleManager();
     void init();
     
     // Вызывается в главном loop() для безопасной обработки пришедших команд
     void process(); 
 
     // Возвращает статус для вывода на OLED-экран
     BleStatus getBleStatus();
 
     // --- Методы отправки данных в Смартфон (Уведомления / Notify) ---
     void sendIdentity(uint8_t nodeId, const char* name, uint8_t role);
     // ИЗМЕНЕНИЕ 1.29: Добавлены таймауты
     void sendSysConfig(uint32_t txMoving, uint32_t txStill, uint32_t connTimeout, uint32_t activeTimeout);
     void sendNodeUpdate(const BleEvtNodeUpdate& nodeData);
     // TODO: void sendMyStatus(...);
 
     // --- Флаги и буферы для чтения в главном loop() ---
     bool hasNewIdentity;
     BleIdentity newIdentity;
 
     bool hasNewSysConfig;
     BleSysConfig newSysConfig;
 
     bool requestFullSync;
     bool requestReset;
     bool requestClearDB;
     // ИЗМЕНЕНИЕ 1.29: Флаги запроса при спаривании
     bool requestIdentitySync;
     bool requestSysConfigSync;
 
 private:
     NimBLEServer* pServer;
     NimBLECharacteristic* pTxCharacteristic;
     NimBLECharacteristic* pRxCharacteristic;
 
     bool _isConnected;
 
     // Внутренние классы обратного вызова (Callbacks) для событий NimBLE
     class ServerCallbacks;
     class RxCallbacks;
 
     friend class ServerCallbacks;
     friend class RxCallbacks;
 };
 
 #endif // BLE_MANAGER_H