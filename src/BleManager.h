/**
 * File: BleManager.h
 * Version: 1.47.1
 * Изменение: Сигнатура sendMyStatus обновлена под новый контракт состояния GPS.
 * Description: Управление BLE-стеком (через библиотеку NimBLE) и реализация протокола Naviga.
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
      void sendNodeDelete(uint8_t nodeId); 
      
      void sendNodeCoords(uint8_t id, float lat, float lon, float snr);
      void sendNodeInfoUpdate(uint8_t id, uint8_t role, const char* name);
 
      // ИЗМЕНЕНИЕ 1.47.1: Первый параметр теперь gpsState вместо gpsValid
      void sendMyStatus(uint8_t gpsState, uint8_t satellites, uint8_t batteryPercent, uint16_t batteryVoltage);
 
      bool hasNewIdentity;
      BleIdentity newIdentity;
      bool hasNewSysConfig;
      BleSysConfig newSysConfig;
      bool requestFullSync;
      bool requestReset;
      bool requestClearDB;
      bool requestIdentitySync;
      bool requestSysConfigSync;
      
      bool hasNewAnchor;
      float newAnchorLat;
      float newAnchorLon;
 
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