/**
 * Project: Naviga-Dongle
 * File: RxManager.cpp
 * Version: 1.43.5
 * Description: Реализация диспетчера входящего эфира.
 */

 #include "RxManager.h"

 // Определяем глобальный аппаратный флаг
 volatile bool receivedFlag = false;
 
 // Реализация ISR
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
     receivedFlag = true;
 }
 
 RxManager::RxManager(RadioManager& radio, Retranslation& router, PacketManager& packetManager,
                      NodeDatabase& nodeDB, TxManager& txManager, BleManager& bleManager,
                      GpsManager& gps, SettingsManager& settingsManager,
                      const uint8_t& myNodeId, const uint8_t& myNodeType, const uint8_t& myMsgSeq,
                      uint32_t& lastHeartbeatTime, CollisionCallback collisionCb)
     : _radio(radio), _router(router), _packetManager(packetManager),
       _nodeDB(nodeDB), _txManager(txManager), _bleManager(bleManager),
       _gps(gps), _settingsManager(settingsManager),
       _myNodeId(myNodeId), _myNodeType(myNodeType), _myMsgSeq(myMsgSeq),
       _lastHeartbeatTime(lastHeartbeatTime), _collisionCb(collisionCb)
 {
 }
 
 bool RxManager::hasNewPacket() const {
     return receivedFlag;
 }
 
 uint32_t RxManager::calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr) {
     uint32_t minMs, maxMs;
     
     if (myRole == NODE_RELAY) {
         minMs = RELAY_JITTER_MIN_MS;
         maxMs = RELAY_JITTER_MAX_MS;
     } else {
         minMs = STALKER_JITTER_MIN_MS;
         maxMs = STALKER_JITTER_MAX_MS;
     }
 
     if (senderRole == NODE_TRACKER) {
         maxMs /= 2;
         if (maxMs < minMs) maxMs = minMs; 
     }
 
     long snrInt = constrain((long)snr, -15, 5);
     uint32_t baseDelay = map(snrInt, -15, 5, minMs, maxMs);
     
     return baseDelay + random(0, 50);
 }
 
 void RxManager::process() {
     // Внимание: проверка receivedFlag дублируется (обычно мы заходим сюда если hasNewPacket == true),
     // но это защита на случай прямого вызова
     if (receivedFlag) {
         noInterrupts(); receivedFlag = false; interrupts(); // Сброс ISR флага
         
         size_t len = _radio.getPacketLength();
         if (len > 0) {
             uint8_t rxBuffer[256];             
             int state = _radio.readData(rxBuffer, len); 
             if (state == RADIOLIB_ERR_NONE) {
                 
                 float currentSNR = _radio.getSNR();
                 
                 if (len >= sizeof(NavigaHeader)) {
                     NavigaHeader rxHeader;
                     memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                     size_t payloadLen = len - sizeof(NavigaHeader);
                     
                     bool isCollision = false;
                     bool isOwnEcho = false;
 
                     if (rxHeader.relayId == _myNodeId) {
                         isCollision = true; 
                     } else if (rxHeader.senderId == _myNodeId) {
                         int8_t seqDiff = (int8_t)(_myMsgSeq - rxHeader.msgSeq);
                         if (seqDiff <= 0 || seqDiff > 10) {
                             isCollision = true; 
                         } else {
                             isOwnEcho = true;   
                         } 
                     } 
 
                     if (isCollision) {
                         if (_collisionCb) _collisionCb();
                     } 
 
                     if (rxHeader.relayId != _myNodeId) {
                         _nodeDB.updateNodeSNR(rxHeader.relayId, currentSNR);
                     } 
 
                     if (isOwnEcho) {
                         // Эхо игнорируем
                     } else if (!_router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d", rxHeader.getType(), payloadLen);
                     } else if (_router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         if (!_nodeDB.hasNodesInOppositeDirection(rxHeader.relayId)) {
                             _txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                         } 
                     } else {
                         LOG_INFO("LORA", "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, SNR: %.1f)", 
                                    rxHeader.getType(), rxHeader.senderId, rxHeader.relayId, rxHeader.msgSeq, currentSNR);
                         
                         bool isNewNode = !_nodeDB.isNodeActive(rxHeader.senderId);
                         _packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen);
 
                         const NodeRecord* updatedNode = _nodeDB.getNode(rxHeader.senderId);
                         if (updatedNode != nullptr) {
                             BleEvtNodeUpdate update;
                             update.opCode = EVT_NODE_UPDATE;
                             update.nodeId = updatedNode->nodeId;
                             update.nodeRole = updatedNode->type;
                             strncpy(update.nodeName, updatedNode->nodeName, sizeof(update.nodeName)-1);
                             update.nodeName[sizeof(update.nodeName)-1] = '\0';
                             update.lat = updatedNode->lat;
                             update.lon = updatedNode->lon;
                             update.snr = updatedNode->snr;
                             update.lastSeenAge = millis() - updatedNode->lastSeen;
                             
                             _bleManager.sendNodeUpdate(update);
                         }
 
                         if (isNewNode && rxHeader.senderId != _myNodeId) {
                             uint32_t currentMillis = millis();
                             uint32_t jitterMs = random(MIN_GREETING_NODEINFO_JITTER, MAX_GREETING_NODEINFO_JITTER); 
                             
                             // Изменяем таймер через переданную ссылку
                             _lastHeartbeatTime = currentMillis - HEARTBEAT_INTERVAL_MS + jitterMs;
                             LOG_INFO("SYS", "New Node %d discovered! NodeInfo reply scheduled", rxHeader.senderId);
                             
                             _settingsManager.saveNodesSnapshot(_nodeDB);
                         } 
 
                         float currentSpeed = _gps.getSpeed();
                         
                         if (_router.shouldRetransmit(rxHeader, _nodeDB, _myNodeType, currentSpeed)) {
                             uint8_t senderRole = NODE_STALKER; 
                             const NodeRecord* senderNode = _nodeDB.getNode(rxHeader.senderId);
                             if (senderNode != nullptr) {
                                 senderRole = senderNode->type;
                             }
                             
                             uint32_t calculatedJitter = calculateRelayJitter(_myNodeType, senderRole, currentSNR);
                             _txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, calculatedJitter);
                         } 
                     } 
                 } 
             } 
         } 
         _radio.startReceive(); 
     } 
 }  //RxManager.cpp