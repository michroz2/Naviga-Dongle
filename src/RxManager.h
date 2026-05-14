/**
 * Project: Naviga-Dongle
 * File: RxManager.h
 * Version: 1.43.5
 * Description: Диспетчер входящего радиоэфира. Изолирует прерывания, 
 * фильтрацию коллизий и принятие решений о маршрутизации.
 */

 #ifndef RX_MANAGER_H
 #define RX_MANAGER_H
 
 #include <Arduino.h>
 #include "configuration.h"
 #include "logger.h"
 #include "RadioManager.h"
 #include "Retranslation.h"
 #include "PacketManager.h"
 #include "NodeDatabase.h"
 #include "TxManager.h"
 #include "BleManager.h"
 #include "GpsManager.h"
 #include "SettingsManager.h"
 
 // Тип коллбэка для обработки коллизий (делегируется в main.cpp)
 typedef void (*CollisionCallback)();
 
 // Экспортируем глобальный флаг прерывания, так как он нужен TxManager (CSMA/CA)
 // и функции первоначального сканирования в main.cpp
 extern volatile bool receivedFlag;
 
 // Функция прерывания (ISR) от LoRa модуля
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void);
 
 class RxManager {
 public:
     RxManager(RadioManager& radio, Retranslation& router, PacketManager& packetManager,
               NodeDatabase& nodeDB, TxManager& txManager, BleManager& bleManager,
               GpsManager& gps, SettingsManager& settingsManager,
               const uint8_t& myNodeId, const uint8_t& myNodeType, const uint8_t& myMsgSeq,
               uint32_t& lastHeartbeatTime, CollisionCallback collisionCb);
 
     // Проверка наличия непрочитанного пакета
     bool hasNewPacket() const;
 
     // Главный цикл разбора и маршрутизации
     void process();
 
 private:
     RadioManager& _radio;
     Retranslation& _router;
     PacketManager& _packetManager;
     NodeDatabase& _nodeDB;
     TxManager& _txManager;
     BleManager& _bleManager;
     GpsManager& _gps;
     SettingsManager& _settingsManager;
 
     // Ссылки на глобальные переменные для отслеживания их изменения в реальном времени
     const uint8_t& _myNodeId;
     const uint8_t& _myNodeType;
     const uint8_t& _myMsgSeq;
     uint32_t& _lastHeartbeatTime; // Передаем по ссылке, чтобы изменять системный таймер
 
     CollisionCallback _collisionCb;
 
     // Внутренняя логика расчета джиттера перенесена сюда из main.cpp
     uint32_t calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr);
 };
 
 #endif // RX_MANAGER_H