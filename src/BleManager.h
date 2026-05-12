/**
 * File: BleManager.h
 * Version: 1.40 
 * Изменение: В методе sendNodeUpdate удалены параметры distance и azimuth (v1.40).
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
     void sendMyStatus(uint8_t gpsValid, uint8_t satellites, uint8_t batteryPercent, uint16_t batteryVoltage); // ИЗМЕНЕНИЕ 1.34
 
     // --- Флаги и буферы ---
     // Выставление флагов при приеме данных. Обработка флагов осуществляется в main.cpp
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
 
     // Дружественные классы для доступа к приватным переменным (особенно _isConnected и флагам)
     friend class ServerCallbacks;
     friend class RxCallbacks;
 };
 
 #endif // BLE_MANAGER_H