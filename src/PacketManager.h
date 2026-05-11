/**
 * File: PacketManager.h
 * Version: 1.38
 * Description: Изолированный класс для парсинга и маршрутизации входящих пакетов.
 * Изменение: Сигнатура processPacket теперь принимает payloadLen для обработки пакетов переменной длины (Шаг 2).
 */
 #ifndef PACKET_MANAGER_H
 #define PACKET_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"
 #include "GpsManager.h"
 #include "GeoPacker.h"
 
 class PacketManager {
 public:
     // Конструктор принимает ссылки на необходимые подсистемы
     PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer);
 
     // Главный метод обработки пакета (вызывается из loop)
     void processPacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen);
     
     // Геттер и сеттер для ID последней цели (для отображения на экране)
     uint8_t getLastTargetId() const;
     void clearLastTargetId();
 
 private:
     NodeDatabase& _nodeDB;
     GpsManager& _gps;
     GeoPacker& _packer;
     uint8_t _lastTargetId;
 
     // Скрытые методы для обработки конкретных типов пакетов
     void handleCoordsPacket(uint8_t senderId, const uint8_t* payload);
     void handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload, size_t payloadLen);
     void handleLeavePacket(uint8_t senderId, const uint8_t* payload);
 };
 
 #endif // PACKET_MANAGER_H