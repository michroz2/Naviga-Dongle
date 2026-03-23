/**
 * File: PacketManager.cpp
 * Version: 1.0.0
 * Description: Реализация класса диспетчера пакетов.
 */
 #include "PacketManager.h"
 #include "logger.h"
 
 PacketManager::PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer)
     : _nodeDB(db), _gps(gps), _packer(packer), _lastTargetId(0) {
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
 
     if (!_gps.isValid()) {
         LOG_WARN("DISPATCH", "No GPS fix. Saved RAW coords for Node %d", senderId);
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
 
 void PacketManager::handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload) {
     PayloadNodeInfo info;
     memcpy(&info, payload, sizeof(PayloadNodeInfo));
     _nodeDB.updateNodeName(senderId, info.nodeName);
     LOG_INFO("DISPATCH", "Updated Name: Node %d -> %s", senderId, info.nodeName);
 } 
 
 void PacketManager::handleLeavePacket(uint8_t senderId, const uint8_t* payload) {
     PayloadLeave info;
     memcpy(&info, payload, sizeof(PayloadLeave)); 
     _nodeDB.removeNode(senderId);
     LOG_INFO("DISPATCH", "Node %d left the network. Reason code: %d", senderId, info.reason);
 } 
 
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
     } 
 }