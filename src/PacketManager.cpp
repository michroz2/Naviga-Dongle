/**
 * File: PacketManager.cpp
 * Version: 1.46
 * Изменение: Использование частичных обновлений ble.sendNodeCoords и ble.sendNodeInfo при обработке LoRa пакетов.
 */

 #include "PacketManager.h"
 #include "logger.h"

 PacketManager::PacketManager(NodeDatabase& nodeDB, GpsManager& gps, GeoPacker& packer, BleManager& ble)
     : _nodeDB(nodeDB), _gps(gps), _packer(packer), _ble(ble), _lastTargetId(0) {}

 void PacketManager::processPacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen) {
     switch (header.getType()) {
         case MSG_COORDS:
             handleCoordsPacket(header.senderId, payload);
             _lastTargetId = header.senderId;
             break;
         case MSG_NODE_INFO:
             handleNodeInfoPacket(header.senderId, payload, payloadLen);
             _lastTargetId = header.senderId;
             break;
         case MSG_LEAVE:
             handleLeavePacket(header.senderId, payload);
             break;
         default:
             LOG_WARN("PKT", "Unknown Radio packet type: %d", header.getType());
             break;
     }
 }

 void PacketManager::handleCoordsPacket(uint8_t senderId, const uint8_t* payload) {
     uint32_t packedCoords;
     memcpy(&packedCoords, payload, 4);
     
     // 1. Обновляем локальную базу (отмечаем получение координат)
     _nodeDB.updateNodeCoords(senderId, 0, 0, packedCoords, true);
     
     // 2. ИЗМЕНЕНИЕ 1.46: Распаковываем и шлем частичный апдейт координат по BLE
     const NodeRecord* node = _nodeDB.getNode(senderId);
     if (node != nullptr) {
         float uLat, uLon;
         _packer.unpack(packedCoords, _gps.getLat(), _gps.getLon(), uLat, uLon);
         _ble.sendNodeCoords(senderId, uLat, uLon, node->snr);
     }
 }

 void PacketManager::handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload, size_t payloadLen) {
     if (payloadLen < 2) return;
     uint8_t role = payload[0];
     char name[24];
     size_t nameLen = payloadLen - 1;
     if (nameLen > 23) nameLen = 23;
     memcpy(name, payload + 1, nameLen);
     name[nameLen] = '\0';

     // 1. Обновляем базу
     _nodeDB.updateNodeInfo(senderId, name, role);

     // 2. ИЗМЕНЕНИЕ 1.46: Шлем частичный апдейт инфо по BLE
     _ble.sendNodeInfo(senderId, role, name);
 }

 void PacketManager::handleLeavePacket(uint8_t senderId, const uint8_t* payload) {
     _nodeDB.removeNode(senderId);
     _ble.sendNodeDelete(senderId); 
     LOG_INFO("DISPATCH", "Node %d left. BLE DELETE sent.", senderId);
 }

 uint8_t PacketManager::getLastTargetId() const { return _lastTargetId; }
 void PacketManager::clearLastTargetId() { _lastTargetId = 0; }
 //PacketManager.cpp