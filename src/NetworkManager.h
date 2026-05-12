/**
 * File: NetworkManager.h
 * Version: 1.00 (Refactored from main.cpp v1.43)
 * Description: Выделенный менеджер управления сетевой логикой: сканирование, коллизии, качество связи.
 */

 #ifndef NETWORK_MANAGER_H
 #define NETWORK_MANAGER_H
 
 #include <Arduino.h>
 #include "RadioManager.h"
 #include "NodeDatabase.h"
 #include "DisplayManager.h"
 #include "GpsManager.h"
 #include "TxManager.h"
 #include "SettingsManager.h"
 #include "BleManager.h"
 #include "Retranslation.h"
 #include "PacketManager.h"
 
 class NetworkManager {
 public:
     NetworkManager(RadioManager& radio, NodeDatabase& db, DisplayManager& disp, 
                   GpsManager& gps, TxManager& tx, BleManager& ble, Retranslation& router, PacketManager& packetMgr);
 
     // Перенесенные функции управления сетью
     void scanNetwork(bool isWarmStart, uint32_t networkScanDuration, uint8_t& myNodeId);
     void handleCollision(uint8_t& myNodeId, uint8_t& myMsgSeq, uint8_t myNodeType);
     uint32_t calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr);
     int getConnectionQuality(uint8_t targetId, uint8_t myNodeId);
 
 private:
     RadioManager& _radio;
     NodeDatabase& _nodeDB;
     DisplayManager& _display;
     GpsManager& _gps;
     TxManager& _txManager;
     BleManager& _bleManager;
     Retranslation& _router;
     PacketManager& _packetManager;
 };
 
 #endif // NETWORK_MANAGER_H