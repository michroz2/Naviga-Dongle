/**
 * File: PacketManager.cpp
 * Version: 1.1.1
 * Description: Реализация диспетчера пакетов.
 * Изменение: Приведение к новым правилам оформления (комментирование скобок).
 */
 #include "PacketManager.h"
 #include "logger.h"
 
 PacketManager::PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer)
     : _nodeDB(db), _gps(gps), _packer(packer), _lastTargetId(0) {
 } // PacketManager::PacketManager()
 
 uint8_t PacketManager::getLastTargetId() const {
     return _lastTargetId;
 } // PacketManager::getLastTargetId()
 
 void PacketManager::clearLastTargetId() {
     _lastTargetId = 0;
 } // PacketManager::clearLastTargetId()
 
 void PacketManager::handleCoordsPacket(uint8_t senderId, const uint8_t* payload) {
     uint32_t packedCoords;
     memcpy(&packedCoords, payload, sizeof(PayloadCoords));
 
     if (!_gps.isValid()) {
         LOG_WARN("DISPATCH", "No GPS fix. Saved RAW coords for Node %d", senderId);
         _nodeDB.updateNodeCoords(senderId, 0.0f, 0.0f, packedCoords);
         _lastTargetId = senderId; 
         return;
     } // if (!_gps.isValid())
 
     float unpLat, unpLon;
     _packer.unpack(packedCoords, _gps.getLat(), _gps.getLon(), unpLat, unpLon);
     
     _nodeDB.updateNodeCoords(senderId, unpLat, unpLon, packedCoords);
     _lastTargetId = senderId; 
     LOG_INFO("DISPATCH", "Extracted COORDS: Node %d -> Lat: %.6f, Lon: %.6f", senderId, unpLat, unpLon);
 } // PacketManager::handleCoordsPacket()
 
 void PacketManager::handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload) {
     PayloadNodeInfo info;
     memcpy(&info, payload, sizeof(PayloadNodeInfo));
     
     _nodeDB.updateNodeInfo(senderId, info.nodeName, info.nodeType);
     LOG_INFO("DISPATCH", "Updated Node %d: Name=%s, Type=%d", senderId, info.nodeName, info.nodeType);
 } // PacketManager::handleNodeInfoPacket()
 
 void PacketManager::handleLeavePacket(uint8_t senderId, const uint8_t* payload) {
     PayloadLeave info;
     memcpy(&info, payload, sizeof(PayloadLeave)); 
     _nodeDB.removeNode(senderId);
     LOG_INFO("DISPATCH", "Node %d left the network. Reason code: %d", senderId, info.reason);
 } // PacketManager::handleLeavePacket()
 
 void PacketManager::processPacket(const NavigaHeader& header, const uint8_t* payload) {
     switch(header.getType()) {
         case MSG_COORDS:
             handleCoordsPacket(header.senderId, payload);
             break;
         case MSG_NODE_INFO:
             handleNodeInfoPacket(header.senderId, payload);
             break;
         case MSG_LEAVE:
             handleLeavePacket(header.senderId, payload);
             break;
         default:
             break;
     } // switch(header.getType())
 } // PacketManager::processPacket()