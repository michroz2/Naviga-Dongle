/**
 * File: BleManager.h
 * Version: 1.46.3
 * Изменение: Прототипы для частичных обновлений. Убран дубликат BleStatus.
 * Description: Заголовочный файл менеджера Bluetooth.
 */

 #ifndef BLE_MANAGER_H
 #define BLE_MANAGER_H
 
 #include <NimBLEDevice.h>
 #include "BleProtocol.h" 
 
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
     void sendMyStatus(uint8_t gpsValid, uint8_t satellites, uint8_t batteryPercent, uint16_t batteryVoltage);
     
     void sendNodeCoords(uint8_t id, float lat, float lon, float snr);
     void sendNodeInfo(uint8_t id, uint8_t role, const char* name);
 
     bool hasNewIdentity;
     BleIdentity newIdentity;
     
     bool hasNewSysConfig;
     BleSysConfig newSysConfig;
     
     bool requestFullSync;
     bool requestReset;
     bool requestClearDB;
     bool requestIdentitySync;
     bool requestSysConfigSync;
 
     char macSuffix[5];
 
 private:
     NimBLEServer* pServer;
     NimBLECharacteristic* pTxCharacteristic;
     NimBLECharacteristic* pRxCharacteristic;
     bool _isConnected;
 
     class ServerCallbacks;
     class RxCallbacks;
 };
 
 #endif // BleManager.h