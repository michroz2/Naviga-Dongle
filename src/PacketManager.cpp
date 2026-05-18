/**
 * File: PacketManager.cpp
 * Version: 1.46.7
 * Изменение: Проверка _gps.hasAnchor() вместо абсолютного фикса спутников при разборе MSG_COORDS.
 * Description: Реализация диспетчера пакетов. Распаковывает Payload и обновляет базу данных.
 */
 #include "PacketManager.h"
 #include "logger.h"
 
 PacketManager::PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer, BleManager& ble)
     : _nodeDB(db), _gps(gps), _packer(packer), _ble(ble), _lastTargetId(0) {
 } 
 
 uint8_t PacketManager::getLastTargetId() const {
     return _lastTargetId;
 } 
 
 void PacketManager::clearLastTargetId() {
     _lastTargetId = 0;
 } 
 
 void PacketManager::handleCoordsPacket(uint8_t senderId, const uint8_t* payload) {
     uint32_t packedCoords;
     memcpy(&packedCoords, payload, sizeof(PayloadCoords));
 
     // ИЗМЕНЕНИЕ 1.46.7: Распаковываем данные, если есть пространственная RAM-опора
     if (!_gps.hasAnchor()) {
         LOG_WARN("DISPATCH", "No GPS anchor available. Saved RAW coords for Node %d", senderId);
         _nodeDB.updateNodeCoords(senderId, 0.0f, 0.0f, packedCoords);
         _lastTargetId = senderId; 
         return;
     } 
 
     float unpLat, unpLon;
     _packer.unpack(packedCoords, _gps.getLat(), _gps.getLon(), unpLat, unpLon);
     
     _nodeDB.updateNodeCoords(senderId, unpLat, unpLon, packedCoords);
     _lastTargetId = senderId; 
     LOG_INFO("DISPATCH", "Extracted COORDS: Node %d -> Lat: %.6f, Lon: %.6f", senderId, unpLat, unpLon);
 } 
 
 void PacketManager::handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload, size_t payloadLen) {
     if (payloadLen < 2) return; 
     
     uint8_t nodeType = payload[0];
     
     char nodeName[24];
     size_t nameLen = payloadLen - 1; 
     if (nameLen > 23) nameLen = 23;  
     
     memcpy(nodeName, payload + 1, nameLen);
     nodeName[nameLen] = '\0'; 
     
     _nodeDB.updateNodeInfo(senderId, nodeName, nodeType);
     LOG_INFO("DISPATCH", "Updated Node %d: Name=%s, Type=%d", senderId, nodeName, nodeType);
 } 
 
 void PacketManager::handleLeavePacket(uint8_t senderId, const uint8_t* payload) {
     PayloadLeave info;
     memcpy(&info, payload, sizeof(PayloadLeave)); 
     
     _nodeDB.removeNode(senderId); 
     
     _ble.sendNodeDelete(senderId);
     
     LOG_INFO("DISPATCH", "Node %d left the network. Reason code: %d", senderId, info.reason);
 } 
 
 void PacketManager::processPacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen) {
     if (!_nodeDB.isNodeActive(header.senderId)) {
         _nodeDB.addNode(header.senderId);
     }
 
     switch(header.getType()) {
         case MSG_COORDS:
             handleCoordsPacket(header.senderId, payload);
             break;
         case MSG_NODE_INFO:
             handleNodeInfoPacket(header.senderId, payload, payloadLen);
             break;
         case MSG_LEAVE:
             handleLeavePacket(header.senderId, payload);
             break;
         default:
             break;
     } 
 }