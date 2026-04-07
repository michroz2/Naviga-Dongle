/**
 * Project: Naviga-Dongle (T-Beam v1.1 Custom E22 + Universal GPS)
 * File: main.cpp
 * Version: 1.17 Изменение: Зависимость updateTopology от gps.isValid() и векторная отмена abortRelay.
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
 #define RED_LED_PIN 4                 

 uint8_t myNodeId = 0; 
 uint8_t myMsgSeq = 0;
 uint8_t myNodeType = NODE_STALKER; 

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
     
     pinMode(RED_LED_PIN, OUTPUT);
     digitalWrite(RED_LED_PIN, LOW); 
     
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
     
     digitalWrite(RED_LED_PIN, HIGH); 
     
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
     pinMode(LORA_ONBOARD_CS, OUTPUT);
     digitalWrite(LORA_ONBOARD_CS, HIGH);
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF); 
     Wire.begin(I2C_SDA, I2C_SCL);                       
     SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
     
     power.init();
     display.init(); 
     display.showLogo();
     gps.init(updateScreenCb, cycleGpsPowerCb); 
     
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

    // НОВОЕ: Обновление топологии строго при наличии фикса GPS
    if (gps.isValid() && (currentMillis - lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS)) {
        nodeDB.updateTopology();
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
                         
                         // НОВОЕ: Умная отмена ретрансляции на основе векторов
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

                         if (router.shouldRetransmit(rxHeader, nodeDB)) {
                             txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, currentSNR);
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
        float currentSpeed = gps.getSpeed();
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

     if (millis() - lastCleanupTime >= NODE_TIMEOUT_MS) {
         lastCleanupTime = millis();
         nodeDB.cleanup(myNodeId); 
     } 

     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
         
         String line1, line2, line3, line4;
         int sats = gps.getSatellites();
 
         uint8_t currentTargetId = packetManager.getLastTargetId();
         const NodeRecord* targetNode = nodeDB.getNode(currentTargetId);
         
         bool isTargetValid = (targetNode != nullptr && targetNode->isActive);

         if (!isTargetValid && currentTargetId != 0) {
             packetManager.clearLastTargetId();
             currentTargetId = 0;
         } 

         if (!gps.isValid()) {
             line1 = (sats > 0) ? ("GPS Wait " + String(sats)) : "GPS ERROR";
         } else {
             line1 = "GPS OK " + String(sats);

             nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0, false);
             
             if (!isLonScaleSet) {
                 packer.updateLonScale(gps.getLat());
                 isLonScaleSet = true;
             } 

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
 
         line2 = "My: " + String(myNodeId) + "-" + String(myMsgSeq);
         
         uint8_t totalNodes = nodeDB.getActiveNodesCount();
         uint8_t neighbors = (totalNodes > 0) ? (totalNodes - 1) : 0;
         line3 = "Neighbors: " + String(neighbors);
         
         if (isTargetValid) {
             line4 = String(targetNode->nodeId) + ": " + 
                     String((int)targetNode->distance) + "m/" + 
                     String((int)targetNode->azimuth) + "/" + 
                     String(getConnectionQuality(targetNode->nodeId));
         } else {
             line4 = "No targets";
         } 

         display.showStatus(line1, line2, line3, line4); 
     } 
 }