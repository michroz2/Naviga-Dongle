/**
 * File: PacketManager.h
 * Version: 1.41
 * Description: Изолированный класс для парсинга и маршрутизации входящих пакетов.
 * Изменение: Добавлена ссылка на BleManager для уведомления Оператора о выходе узлов (v1.41).
 */
 #ifndef PACKET_MANAGER_H
 #define PACKET_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"
 #include "GpsManager.h"
 #include "GeoPacker.h"
 #include "BleManager.h" // ИЗМЕНЕНИЕ 1.41
 
 class PacketManager {
 public:
     // ИЗМЕНЕНИЕ 1.41: Конструктор теперь принимает ссылку на BleManager
     PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer, BleManager& ble);
 
     void processPacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen);
     
     uint8_t getLastTargetId() const;
     void clearLastTargetId();
 
 private:
     NodeDatabase& _nodeDB;
     GpsManager& _gps;
     GeoPacker& _packer;
     BleManager& _ble; // ИЗМЕНЕНИЕ 1.41
     uint8_t _lastTargetId;
 
     void handleCoordsPacket(uint8_t senderId, const uint8_t* payload);
     void handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload, size_t payloadLen);
     void handleLeavePacket(uint8_t senderId, const uint8_t* payload);
 };
 
 #endif // PACKET_MANAGER_H