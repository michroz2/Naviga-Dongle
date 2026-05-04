/**
 * File: BleManager.h
 * Version: 1.32 Изменение: Добавлена переменная macSuffix для хранения суффикса устройства.
 * Description: Управление BLE-стеком (NimBLE) и реализация протокола Naviga (UC-04).
 */

 #ifndef BLE_MANAGER_H
 #define BLE_MANAGER_H
 
 #include <Arduino.h>
 #include <NimBLEDevice.h>
 #include "BleConfig.h"
 #include "BleProtocol.h"
 #include "DisplayManager.h" 
 
 class BleManager {
 public:
     BleManager();
     void init();
     
     void process(); 
     BleStatus getBleStatus();

     void sendIdentity(uint8_t nodeId, const char* name, uint8_t role);
     void sendSysConfig(uint32_t txMoving, uint32_t txStill, uint32_t connTimeout, uint32_t activeTimeout);
     void sendNodeUpdate(const BleEvtNodeUpdate& nodeData);
 
     // --- Флаги и буферы ---
     bool hasNewIdentity;
     BleIdentity newIdentity;
     bool hasNewSysConfig;
     BleSysConfig newSysConfig;
     bool requestFullSync;
     bool requestReset;
     bool requestClearDB;
     bool requestIdentitySync;
     bool requestSysConfigSync;

     // ИЗМЕНЕНИЕ 1.32: Храним 4 символа MAC-адреса
     char macSuffix[5]; 

 private:
     NimBLEServer* pServer;
     NimBLECharacteristic* pTxCharacteristic;
     NimBLECharacteristic* pRxCharacteristic;
 
     bool _isConnected;
 
     class ServerCallbacks;
     class RxCallbacks;
 
     friend class ServerCallbacks;
     friend class RxCallbacks;
 };
 
 #endif // BLE_MANAGER_H