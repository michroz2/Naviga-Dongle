/**
 * File: BleManager.h
 * Version: 1.46.5
 * Изменение: Добавлены методы дельта-обновлений sendNodeCoords и sendNodeInfoUpdate.
 * Description: Управление BLE-стеком (через библиотеку NimBLE) и реализация протокола Naviga (UC-04).
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
     void init(); // Инициализация BLE-сервера и рекламных пакетов
     
     void process(); // Оставлен для совместимости (логика перенесена в main)
     BleStatus getBleStatus();
 
     // Методы отправки данных (NOTIFY) в смартфон
     void sendIdentity(uint8_t nodeId, const char* name, uint8_t role);
     void sendSysConfig(uint32_t txMoving, uint32_t txStill, uint32_t connTimeout, uint32_t activeTimeout);
     void sendNodeUpdate(const BleEvtNodeUpdate& nodeData);
     void sendNodeDelete(uint8_t nodeId); 
     
     // НОВОЕ 1.46.5: Методы оптимизации трафика (Дельта-пакеты)
     void sendNodeCoords(uint8_t id, float lat, float lon, float snr);
     void sendNodeInfoUpdate(uint8_t id, uint8_t role, const char* name);
 
     void sendMyStatus(uint8_t gpsValid, uint8_t satellites, uint8_t batteryPercent, uint16_t batteryVoltage);
 
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
 
     // Храним 4 символа аппаратного MAC-адреса для вывода на экран и в имя
     char macSuffix[5]; 
 
 private:
     NimBLEServer* pServer;                     // Указатель на BLE-сервер
     NimBLECharacteristic* pTxCharacteristic;   // Характеристика передачи (NOTIFY)
     NimBLECharacteristic* pRxCharacteristic;   // Характеристика приема (WRITE)
 
     bool _isConnected; // Состояние подключения смартфона
 
     // Вложенные классы для обработки событий (коллбэков) от библиотеки NimBLE
     class ServerCallbacks;
     class RxCallbacks;
 
     // Дружественные классы для доступа к приватным переменным
     friend class ServerCallbacks;
     friend class RxCallbacks;
 };
 
 #endif // BLE_MANAGER_H