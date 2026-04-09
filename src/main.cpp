/**
 * Project: Naviga-Dongle (T-Beam v1.1 / T-Energy S3 + Custom E22 + GPS)
 * File: main.cpp
 * Version: 1.24 Изменение: Внедрен жесткий аппаратный сброс LoRa до запуска SPI (Защита от глитчей).
 * Description: Главный файл оркестратора.
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <SPI.h>              
 #include "configuration.h"
 #include "logger.h"           
 #include "GeoPacker.h"        
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"     
 #include "Retranslation.h"    
 #include "GpsManager.h"       
 #include "DisplayManager.h"   
 #include "RadioManager.h"     
 #include "PowerManager.h"     
 #include "PacketManager.h"    
 #include "TxManager.h"        
 
 // --- НАСТРОЙКИ СКАНИРОВАНИЯ ---
 uint32_t networkScanDuration = 30000; 
 
 uint8_t myNodeId = 0; 
 uint8_t myMsgSeq = 0;
 uint8_t myNodeType = NODE_RELAY; // Ролевая модель (NODE_TRACKER, NODE_STALKER, NODE_RELAY)
 
 PowerManager power;                                      
 DisplayManager display(0x3c, I2C_SDA, I2C_SCL); 
 GpsManager gps;                                       
 RadioManager radio; 
 GeoPacker packer;                                      
 NodeDatabase nodeDB;                                   
 Retranslation router;                                  
 PacketManager packetManager(nodeDB, gps, packer);
 TxManager txManager(radio, packer, myNodeId, myMsgSeq);
 
 volatile bool receivedFlag = false;                    
 
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
     receivedFlag = true;
 } 
 
 uint32_t lastTxTime = 0;                               
 uint32_t lastGpsLogTime = 0;                           
 uint32_t lastCleanupTime = 0;                                
 uint32_t lastHeartbeatTime = 0;    
 uint32_t lastTopologyUpdateTime = 0;
 
 bool isLonScaleSet = false;                            
 
 // Единый вычислитель Джиттера с учетом ролей (v1.19)
 uint32_t calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr) {
     uint32_t minMs, maxMs;
     
     if (myRole == NODE_RELAY) {
         minMs = RELAY_JITTER_MIN_MS;
         maxMs = RELAY_JITTER_MAX_MS;
     } else {
         minMs = STALKER_JITTER_MIN_MS;
         maxMs = STALKER_JITTER_MAX_MS;
     }
 
     // VIP-пакет (от Трекера): сдвигаем окно вниз
     if (senderRole == NODE_TRACKER) {
         maxMs /= 2;
         if (maxMs < minMs) maxMs = minMs; 
     }
 
     long snrInt = constrain((long)snr, -15, 5);
     uint32_t baseDelay = map(snrInt, -15, 5, minMs, maxMs);
     
     return baseDelay + random(0, 50);
 }
 
 int getConnectionQuality(uint8_t targetId) {
     if (targetId == myNodeId) return 10; 
 
     const NodeRecord* target = nodeDB.getNode(targetId);
     if (target == nullptr || targetId == 0) return 0;
     if (millis() - target->lastSeen > 30000) return 0;
     
     if (target->snr <= -99.0f) return 0; 
 
     int q = map((long)target->snr, -11, 5, 1, 10);
     if (q < 1) q = 1;
     if (q > 10) q = 10;
     return q;
 } 
 
 void updateScreenCb(String line1, String line2, String line3, String line4) {
     display.showStatus(line1, line2, line3, line4);
 } 
 
 void cycleGpsPowerCb() {
     power.cycleGpsPower();
 } 
 
 void handleCollision() {
    uint8_t oldId = myNodeId;
    nodeDB.removeNode(oldId); 
    
    randomSeed(esp_random());
    do {
        myNodeId = random(1, 255);
    } while (nodeDB.getNode(myNodeId) != nullptr); 
    
    nodeDB.addNode(myNodeId); 
    
    LOG_WARN("COLLISION", "ID %d is taken! Switched to new ID: %d", oldId, myNodeId);
    
    myMsgSeq = 0; 
    
    char myName[12];
    snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
    
    txManager.sendNodeInfo(myName, myNodeType, TX_CRITICAL);
    nodeDB.updateNodeInfo(myNodeId, myName, myNodeType); 
 } 
 
 void scanNetworkForUniqueId() {
     LOG_INFO("SYS", "Starting network scan for %d ms...", networkScanDuration);
     
     display.toggleLed();
     
     uint32_t scanStart = millis();
     uint32_t lastDispUpdate = 0;
     
     receivedFlag = false;
     radio.startReceive(); 
     
     while (millis() - scanStart < networkScanDuration) {
         uint32_t now = millis();
         
         if (now - lastDispUpdate >= 1000) {
             lastDispUpdate = now;
             uint32_t left = (networkScanDuration - (now - scanStart)) / 1000;
             display.showStatus("Scanning Net...", "Time left: " + String(left) + " s", "Nodes Found: " + String(nodeDB.getActiveNodesCount()), "Please wait...");
         } 
         
         if (receivedFlag) {
             noInterrupts(); receivedFlag = false; interrupts();
             
             size_t len = radio.getPacketLength();
             if (len >= sizeof(NavigaHeader)) {
                 uint8_t rxBuffer[256];             
                 int state = radio.readData(rxBuffer, len); 
                 if (state == RADIOLIB_ERR_NONE) {
                     NavigaHeader rxHeader;
                     memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                     size_t payloadLen = len - sizeof(NavigaHeader);
                     
                     if (router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         if (!router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                             packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));
                         } 
                     } 
                 } 
             } 
             radio.startReceive();
         } 
         
         gps.update(); 
     } 
     
     display.toggleLed();
     
     randomSeed(esp_random());
     do {
         myNodeId = random(1, 255);
     } while (nodeDB.getNode(myNodeId) != nullptr); 
     
     nodeDB.addNode(myNodeId); 
     
     LOG_INFO("SYS", "Scan complete. Selected unique Node ID: %d", myNodeId);
     display.showStatus("Scan Complete", "My new ID:", String(myNodeId), "Starting...");
     delay(2000);
 } 
 
 void setup() {
     delay(500); 
     Serial.begin(115200);
     unsigned long start = millis();
     while (!Serial && (millis() - start < 3000)); 
     LOG_INFO("SYS", "--- DONGLE BOOT START ---");
     
     #ifdef BOARD_T_BEAM_V11
     pinMode(LORA_ONBOARD_CS, OUTPUT);
     digitalWrite(LORA_ONBOARD_CS, HIGH);
     #endif
 
     // ИЗМЕНЕНИЕ 1.24: Жесткий аппаратный сброс LoRa-модуля ДО инициализации SPI.
     // Сбрасывает конечный автомат SX1268, предотвращая зависание из-за глитчей на шине.
     pinMode(LORA_RST, OUTPUT);
     digitalWrite(LORA_RST, LOW);
     delay(20);  
     digitalWrite(LORA_RST, HIGH);
     delay(50);  
 
     Wire.begin(I2C_SDA, I2C_SCL);                       
     SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
     
     power.init();
     display.init(); // Инициализирует OLED и LED (если HAS_STATUS_LED)
     display.showLogo();
     gps.init(updateScreenCb, cycleGpsPowerCb); 
     
     // Активация статических координат для Ретранслятора (v1.19)
     if (myNodeType == NODE_RELAY) {
         gps.setStaticLocation(RELAY_STATIC_LAT, RELAY_STATIC_LON);
     }
 
     display.showStatus("System Init...", "GPS Init Done", "Init LoRa...", "");
     if (!radio.init(setFlag)) {
         display.showStatus("ERROR", "LoRa Init Failed", "Check Logs", "");
         delay(3000);
     } 
 
     scanNetworkForUniqueId();
     
     char myName[12];
     snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
     txManager.sendNodeInfo(myName, myNodeType, TX_NORMAL);
     nodeDB.updateNodeInfo(myNodeId, myName, myNodeType); 
     
     lastTxTime = millis(); 
 } 
 
 void loop() {
    uint32_t currentMillis = millis();
    float currentSpeed = gps.getSpeed();
 
    // Флаг для отключения тяжелой математики, если Трекер бежит (v1.18)
    bool isFastTracker = (myNodeType == NODE_TRACKER && currentSpeed > TRACKER_FAST_SPEED_KMPH);
 
    // --- Обновление топологии по таймеру ---
    if (gps.isValid() && (currentMillis - lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS)) {
        if (isFastTracker) {
            LOG_INFO("SYS", "Topology sync skipped: Tracker is running.");
        } else {
            nodeDB.updateTopology();
        }
        lastTopologyUpdateTime = currentMillis;
    }
 
    if (currentMillis - lastCleanupTime > CLEANUP_INTERVAL_MS) {
        nodeDB.cleanup(myNodeId);
        lastCleanupTime = currentMillis;
    } 
 
    if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
        if (myNodeId != 0) { 
            char currentName[12];
            snprintf(currentName, sizeof(currentName), "Node-%d", myNodeId);
            txManager.sendNodeInfo(currentName, myNodeType, TX_NORMAL);
            nodeDB.updateNodeInfo(myNodeId, currentName, myNodeType); 
            LOG_INFO("ACTION", "Heartbeat sent: NodeInfo (Name: %s, Type: %d)", currentName, myNodeType);
        } 
        lastHeartbeatTime = currentMillis;
    } 
 
     gps.update();
 
     if (receivedFlag) {
         noInterrupts(); receivedFlag = false; interrupts();
         
         size_t len = radio.getPacketLength();
         if (len > 0) {
             uint8_t rxBuffer[256];             
             int state = radio.readData(rxBuffer, len); 
             if (state == RADIOLIB_ERR_NONE) {
                 
                 float currentSNR = radio.getSNR();
                 
                 if (len >= sizeof(NavigaHeader)) {
                     NavigaHeader rxHeader;
                     memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                     size_t payloadLen = len - sizeof(NavigaHeader);
                     
                     bool isCollision = false;
                     bool isOwnEcho = false;
 
                     if (rxHeader.relayId == myNodeId) {
                         isCollision = true;
                         LOG_WARN("LORA", "Collision Type 1: Relay ID == myNodeId!");
                     } else if (rxHeader.senderId == myNodeId) {
                         int8_t seqDiff = (int8_t)(myMsgSeq - rxHeader.msgSeq);
                         if (seqDiff <= 0 || seqDiff > 10) {
                             isCollision = true;
                             LOG_WARN("LORA", "Collision Type 2: Seq %d vs my %d (diff: %d)", rxHeader.msgSeq, myMsgSeq, seqDiff);
                         } else {
                             LOG_INFO("LORA", "Valid echo of our pkt Seq %d", rxHeader.msgSeq);
                             isOwnEcho = true;
                         } 
                     } 
 
                     if (isCollision) {
                         handleCollision();
                     } 
 
                     if (rxHeader.relayId != myNodeId) {
                         nodeDB.updateNodeSNR(rxHeader.relayId, currentSNR);
                     } 
 
                     if (isOwnEcho) {
                         // do nothing
                     } else if (!router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d", rxHeader.getType(), payloadLen);
                     } else if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         
                         // Умная отмена ретрансляции на основе векторов
                         if (!nodeDB.hasNodesInOppositeDirection(rxHeader.relayId)) {
                             LOG_INFO("QUEUE", "Relayer %d has no opposite nodes. Aborting our relay job for Seq %d.", rxHeader.relayId, rxHeader.msgSeq);
                             txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                         } else {
                             LOG_INFO("QUEUE", "Nodes exist opposite to relayer %d. Keeping our relay job for Seq %d.", rxHeader.relayId, rxHeader.msgSeq);
                         }
                         
                     } else {
                         LOG_INFO("LORA", "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, SNR: %.1f)", 
                                  rxHeader.getType(), rxHeader.senderId, rxHeader.relayId, rxHeader.msgSeq, currentSNR);
                         
                            bool isNewNode = !nodeDB.isNodeActive(rxHeader.senderId);
                            packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));
 
                            if (isNewNode && rxHeader.senderId != myNodeId) {
                                uint32_t currentMillis = millis();
                                uint32_t jitterMs = random(MIN_GREETING_NODEINFO_JITTER, MAX_GREETING_NODEINFO_JITTER); 
                                lastHeartbeatTime = currentMillis - HEARTBEAT_INTERVAL_MS + jitterMs;
                                LOG_INFO("SYS", "New Node %d discovered! NodeInfo reply (batching) scheduled in %d sec", rxHeader.senderId, jitterMs / 1000);
                            } 
 
                         if (router.shouldRetransmit(rxHeader, nodeDB, myNodeType, currentSpeed)) {
                             uint8_t senderRole = NODE_STALKER; 
                             const NodeRecord* senderNode = nodeDB.getNode(rxHeader.senderId);
                             if (senderNode != nullptr) {
                                 senderRole = senderNode->type;
                             }
                             
                             // Расчет джиттера до помещения в очередь (v1.19)
                             uint32_t calculatedJitter = calculateRelayJitter(myNodeType, senderRole, currentSNR);
                             txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, calculatedJitter);
                         } 
                     } 
                 } 
             } 
         } 
         radio.startReceive();
     } 
 
    if (gps.isValid()) {
        bool shouldTransmit = false;
        const NodeRecord* myRecord = nodeDB.getNode(myNodeId);
        
        float distFromLastTx = (myRecord != nullptr) ? myRecord->distance : 0.0f;
        uint32_t now = millis();
 
        if (distFromLastTx > MIN_MOVEMENT_METERS && currentSpeed > MIN_SPEED_KMPH) {
            shouldTransmit = true;
        } else if (distFromLastTx > SNEAK_MOVEMENT_METERS) {
            shouldTransmit = true;
        } 
 
        if (shouldTransmit && (now - lastTxTime >= TX_INTERVAL_MOVING)) {
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0); 
            LOG_INFO("ACTION", "Adaptive TX (Moving): Dist: %.1fm, Speed: %.1fkm/h", distFromLastTx, currentSpeed);
            lastTxTime = now;
        } else if (now - lastTxTime >= TX_INTERVAL_STILL) {
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0);
            LOG_INFO("ACTION", "Adaptive TX (Still Heartbeat)");
            lastTxTime = now;
        } 
    } else {
        if (millis() - lastTxTime >= TX_INTERVAL_STILL) {
            LOG_WARN("TX", "Skip TX: GPS location not valid.");
            lastTxTime = millis(); 
        } 
    } 
 
     txManager.processQueue();
 
     // --- Секундный блок (Пинг экрана и пересчет геометрии) ---
     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         
         display.toggleLed(); // Инкапсулированный вызов (v1.20)
         
         int sats = gps.getSatellites();
 
         uint8_t currentTargetId = packetManager.getLastTargetId();
         const NodeRecord* targetNode = nodeDB.getNode(currentTargetId);
         
         bool isTargetValid = (targetNode != nullptr && targetNode->isActive);
 
         if (!isTargetValid && currentTargetId != 0) {
             packetManager.clearLastTargetId();
             currentTargetId = 0;
         } 
 
         if (gps.isValid()) {
             if (!isLonScaleSet) {
                 packer.updateLonScale(gps.getLat());
                 isLonScaleSet = true;
             } 
 
             if (!isFastTracker) {
                 for (int i = 1; i < 255; i++) {
                     const NodeRecord* node = nodeDB.getNode(i);
                     if (node != nullptr && node->isActive) {
                         
                         if (node->packedCoords != 0 && node->lat == 0.0f && node->lon == 0.0f) {
                             float unpLat, unpLon;
                             packer.unpack(node->packedCoords, gps.getLat(), gps.getLon(), unpLat, unpLon);
                             nodeDB.updateNodeCoords(i, unpLat, unpLon, node->packedCoords, false);
                         } 
                         
                         if (node->lat != 0.0f || node->lon != 0.0f) {
                             float d = gps.distanceTo(node->lat, node->lon);
                             float a = gps.courseTo(node->lat, node->lon);
                             nodeDB.updateNodeDistanceAzimuth(i, d, a);
                         } 
                     } 
                 } 
             } 
         } 
 
         // Делегирование сборки и вывода UI (v1.20)
         int targetDist = isTargetValid ? (int)targetNode->distance : 0;
         int targetAzimuth = isTargetValid ? (int)targetNode->azimuth : 0;
         int targetQuality = isTargetValid ? getConnectionQuality(targetNode->nodeId) : 0;
 
         display.updateMainScreen(gps.isValid(), sats, myNodeId, myMsgSeq, 
                                  nodeDB.getActiveNodesCount(), isTargetValid, currentTargetId, 
                                  targetDist, targetAzimuth, targetQuality); 
     } 
 }