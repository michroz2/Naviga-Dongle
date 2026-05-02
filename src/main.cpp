/**
 * Project: Naviga-Dongle (T-Beam v1.1 / T-Energy S3 + Custom E22 + GPS)
 * File: main.cpp
 * Version: 1.28 
 * Изменение: Возвращен немой период (Silent Mode) для Warm Start и искусственное старение базы (UC-03).
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
 #include "BleManager.h"       
 #include "SettingsManager.h"  
 
 // --- НАСТРОЙКИ СКАНИРОВАНИЯ ---
 uint32_t networkScanDuration = 30000; 
 
 uint8_t myNodeId = 0; 
 uint8_t myMsgSeq = 0;
 uint8_t myNodeType = NODE_RELAY; 
 
 PowerManager power;                                      
 DisplayManager display(0x3c, I2C_SDA, I2C_SCL); 
 GpsManager gps;                                       
 RadioManager radio; 
 GeoPacker packer;                                      
 NodeDatabase nodeDB;                                   
 Retranslation router;                                  
 PacketManager packetManager(nodeDB, gps, packer);
 TxManager txManager(radio, packer, myNodeId, myMsgSeq);
 BleManager bleManager;                                 
 
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
 
 uint32_t calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr) {
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
 
 int getConnectionQuality(uint8_t targetId) {
     if (targetId == myNodeId) return 10; 
 
     const NodeRecord* target = nodeDB.getNode(targetId);
     if (target == nullptr || targetId == 0) return 0;
     
     if (millis() - target->lastSeen > settingsManager.settings.nodeConnectionTimeout) return 0;
     
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
 
    settingsManager.settings.nodeId = myNodeId;
    settingsManager.save();
    settingsManager.saveNodesSnapshot(nodeDB);
 } 
 
// ИЗМЕНЕНИЕ 1.28: Универсальная функция сканирования/немого периода
void scanNetwork(bool isWarmStart) {
    if (isWarmStart) {
        LOG_INFO("SYS", "Warm Start: Silent listening for %d ms...", networkScanDuration);
    } else {
        LOG_INFO("SYS", "Cold Start: Scanning for %d ms...", networkScanDuration);
    }
    
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
            display.showStatus(isWarmStart ? "Silent Mode..." : "Scanning Net...", 
                               "Time left: " + String(left) + " s", 
                               "Nodes Found: " + String(nodeDB.getActiveNodesCount()), 
                               "Please wait...");
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
    
    // Если это Cold Start (или у нас почему-то нет ID), генерируем новый
    if (!isWarmStart || myNodeId == 0) {
        randomSeed(esp_random());
        do {
            myNodeId = random(1, 255);
        } while (nodeDB.getNode(myNodeId) != nullptr); 
        
        nodeDB.addNode(myNodeId); 
        LOG_INFO("SYS", "Scan complete. Selected unique Node ID: %d", myNodeId);
    } else {
        LOG_INFO("SYS", "Silent listening complete. Kept Node ID: %d", myNodeId);
    }
    
    display.showStatus("Scan Complete", "My ID:", String(myNodeId), "Starting...");
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
 
     pinMode(LORA_RST, OUTPUT);
     digitalWrite(LORA_RST, LOW);
     delay(20);  
     digitalWrite(LORA_RST, HIGH);
     delay(50);  
 
     Wire.begin(I2C_SDA, I2C_SCL);                       
     SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
     
     power.init();
     display.init(); 
     display.showLogo();
     gps.init(updateScreenCb, cycleGpsPowerCb); 
 
     bleManager.init();
     
     settingsManager.init();
     myNodeId = settingsManager.settings.nodeId;
     myNodeType = settingsManager.settings.nodeType;
 
     settingsManager.loadNodesSnapshot(nodeDB);
 
     // ИЗМЕНЕНИЕ 1.28: Старим все восстановленные узлы, чтобы они стали "серыми" (timeout + 1 секунда)
     nodeDB.ageAllNodes(settingsManager.settings.nodeConnectionTimeout + 1000);
 
     // Гарантируем, что наш локальный узел свежий и активный поверх слепка
     char myName[12];
     strncpy(myName, settingsManager.settings.nodeName, sizeof(myName)-1);
     myName[sizeof(myName)-1] = '\0';
     
     nodeDB.addNode(myNodeId);
     nodeDB.updateNodeInfo(myNodeId, myName, myNodeType);
 
     if (myNodeType == NODE_RELAY) {
         gps.setStaticLocation(RELAY_STATIC_LAT, RELAY_STATIC_LON);
     }
 
     display.showStatus("System Init...", "GPS Init Done", "Init LoRa...", "");
     if (!radio.init(setFlag)) {
         display.showStatus("ERROR", "LoRa Init Failed", "Check Logs", "");
         delay(3000);
     } 
 
     // ИЗМЕНЕНИЕ 1.28: Вызов универсальной функции сканирования (Cold vs Warm)
     if (myNodeId == 0 || !settingsManager.settings.isConfigured) {
         scanNetwork(false); // Cold Start
         settingsManager.settings.nodeId = myNodeId;
         settingsManager.settings.isConfigured = true;
         settingsManager.save();
     } else {
         scanNetwork(true);  // Warm Start (Silent period)
     }
     
     txManager.sendNodeInfo(myName, myNodeType, TX_NORMAL);
     
     lastTxTime = millis(); 
 } 
 
 void loop() {
     uint32_t currentMillis = millis();
     float currentSpeed = gps.getSpeed();
 
// ==========================================================
    // ОБРАБОТКА КОМАНД ОТ СМАРТФОНА
    // ==========================================================
    
    // ИЗМЕНЕНИЕ 1.29: Обработка запросов настроек (UC-04 Pairing)
    if (bleManager.requestIdentitySync) {
        bleManager.requestIdentitySync = false;
        bleManager.sendIdentity(
            settingsManager.settings.nodeId, 
            settingsManager.settings.nodeName, 
            settingsManager.settings.nodeType
        );
        LOG_INFO("BLE", "Sent Identity config to Smartphone");
    }

    if (bleManager.requestSysConfigSync) {
        bleManager.requestSysConfigSync = false;
        bleManager.sendSysConfig(
            settingsManager.settings.txIntervalMoving,
            settingsManager.settings.txIntervalStill,
            settingsManager.settings.nodeConnectionTimeout,
            settingsManager.settings.nodeActiveTimeoutMs
        );
        LOG_INFO("BLE", "Sent System Config to Smartphone");
    }
    
    if (bleManager.requestFullSync) {
         bleManager.requestFullSync = false;
         for (int i = 1; i < 255; i++) {
             const NodeRecord* node = nodeDB.getNode(i);
             if (node != nullptr && node->isActive) {
                 BleEvtNodeUpdate update;
                 update.opCode = EVT_NODE_UPDATE;
                 update.nodeId = node->nodeId;
                 update.nodeRole = node->type;
                 strncpy(update.nodeName, node->nodeName, sizeof(update.nodeName)-1);
                 update.nodeName[sizeof(update.nodeName)-1] = '\0';
                 update.lat = node->lat;
                 update.lon = node->lon;
                 update.distance = node->distance;
                 update.azimuth = node->azimuth;
                 update.snr = node->snr;
                 update.lastSeenAge = millis() - node->lastSeen;
                 
                 bleManager.sendNodeUpdate(update);
                 delay(5); 
             }
         }
         LOG_INFO("BLE", "Full topology sync sent to Smartphone");
     }
 
     if (bleManager.hasNewIdentity) {
         bleManager.hasNewIdentity = false;
         myNodeId = bleManager.newIdentity.myNodeId;
         myNodeType = bleManager.newIdentity.myRole;
         
         settingsManager.settings.nodeId = myNodeId;
         settingsManager.settings.nodeType = myNodeType;
         strncpy(settingsManager.settings.nodeName, bleManager.newIdentity.myName, 11);
         settingsManager.save();
 
         txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType, TX_CRITICAL);
         nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName, myNodeType);
         
         settingsManager.saveNodesSnapshot(nodeDB);
         LOG_INFO("BLE", "Identity updated from App and saved (ID: %d)", myNodeId);
     }
 
     if (bleManager.hasNewSysConfig) {
        bleManager.hasNewSysConfig = false;
        settingsManager.settings.txIntervalMoving = bleManager.newSysConfig.txIntervalMoving;
        settingsManager.settings.txIntervalStill = bleManager.newSysConfig.txIntervalStill;
        
        // ИЗМЕНЕНИЕ 1.29: Сохраняем новые таймауты
        settingsManager.settings.nodeConnectionTimeout = bleManager.newSysConfig.nodeConnectionTimeout;
        settingsManager.settings.nodeActiveTimeoutMs = bleManager.newSysConfig.nodeActiveTimeoutMs;
        
        settingsManager.save();
        LOG_INFO("BLE", "SysConfig updated from App and saved");
    }
     
     if (bleManager.requestClearDB) {
         bleManager.requestClearDB = false;
         for (int i = 1; i < 255; i++) {
             if (i != myNodeId) {
                 nodeDB.removeNode(i);
             }
         }
         settingsManager.saveNodesSnapshot(nodeDB); 
         LOG_INFO("BLE", "Node database cleared via App command");
     }
 
     if (bleManager.requestReset) {
         LOG_INFO("BLE", "Executing Hard Reset via App Command...");
         delay(500);
         ESP.restart();
     }
     // ==========================================================
 
    bool isFastTracker = (myNodeType == NODE_TRACKER && currentSpeed > TRACKER_FAST_SPEED_KMPH);
 
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
            txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType, TX_NORMAL);
            nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName, myNodeType); 
            LOG_INFO("ACTION", "Heartbeat sent: NodeInfo");
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
                     } else if (rxHeader.senderId == myNodeId) {
                         int8_t seqDiff = (int8_t)(myMsgSeq - rxHeader.msgSeq);
                         if (seqDiff <= 0 || seqDiff > 10) {
                             isCollision = true;
                         } else {
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
                     } else if (!router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d", rxHeader.getType(), payloadLen);
                     } else if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         
                         if (!nodeDB.hasNodesInOppositeDirection(rxHeader.relayId)) {
                             txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                         } 
                         
                     } else {
                         LOG_INFO("LORA", "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, SNR: %.1f)", 
                                  rxHeader.getType(), rxHeader.senderId, rxHeader.relayId, rxHeader.msgSeq, currentSNR);
                         
                            bool isNewNode = !nodeDB.isNodeActive(rxHeader.senderId);
                            packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));
 
                            const NodeRecord* updatedNode = nodeDB.getNode(rxHeader.senderId);
                            if (updatedNode != nullptr) {
                                BleEvtNodeUpdate update;
                                update.opCode = EVT_NODE_UPDATE;
                                update.nodeId = updatedNode->nodeId;
                                update.nodeRole = updatedNode->type;
                                strncpy(update.nodeName, updatedNode->nodeName, sizeof(update.nodeName)-1);
                                update.nodeName[sizeof(update.nodeName)-1] = '\0';
                                update.lat = updatedNode->lat;
                                update.lon = updatedNode->lon;
                                update.distance = updatedNode->distance;
                                update.azimuth = updatedNode->azimuth;
                                update.snr = updatedNode->snr;
                                update.lastSeenAge = millis() - updatedNode->lastSeen;
                                
                                bleManager.sendNodeUpdate(update);
                            }
 
                            if (isNewNode && rxHeader.senderId != myNodeId) {
                                uint32_t currentMillis = millis();
                                uint32_t jitterMs = random(MIN_GREETING_NODEINFO_JITTER, MAX_GREETING_NODEINFO_JITTER); 
                                lastHeartbeatTime = currentMillis - HEARTBEAT_INTERVAL_MS + jitterMs;
                                LOG_INFO("SYS", "New Node %d discovered! NodeInfo reply scheduled", rxHeader.senderId);
                                
                                settingsManager.saveNodesSnapshot(nodeDB);
                            } 
 
                         if (router.shouldRetransmit(rxHeader, nodeDB, myNodeType, currentSpeed)) {
                             uint8_t senderRole = NODE_STALKER; 
                             const NodeRecord* senderNode = nodeDB.getNode(rxHeader.senderId);
                             if (senderNode != nullptr) {
                                 senderRole = senderNode->type;
                             }
                             
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
 
        if (shouldTransmit && (now - lastTxTime >= settingsManager.settings.txIntervalMoving)) {
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0); 
            LOG_INFO("ACTION", "Adaptive TX (Moving): Dist: %.1fm", distFromLastTx);
            lastTxTime = now;
        } else if (now - lastTxTime >= settingsManager.settings.txIntervalStill) {
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0);
            LOG_INFO("ACTION", "Adaptive TX (Still Heartbeat)");
            lastTxTime = now;
        } 
    } else {
        if (millis() - lastTxTime >= settingsManager.settings.txIntervalStill) {
            LOG_WARN("TX", "Skip TX: GPS location not valid.");
            lastTxTime = millis(); 
        } 
    } 
 
     txManager.processQueue();
 
     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         
         display.toggleLed(); 
         
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
 
         int targetDist = isTargetValid ? (int)targetNode->distance : 0;
         int targetAzimuth = isTargetValid ? (int)targetNode->azimuth : 0;
         int targetQuality = isTargetValid ? getConnectionQuality(targetNode->nodeId) : 0;
 
         BleStatus currentBleStatus = bleManager.getBleStatus();
 
         display.updateMainScreen(gps.isValid(), sats, myNodeId, myMsgSeq, 
                                  nodeDB.getActiveNodesCount(), isTargetValid, currentTargetId, 
                                  targetDist, targetAzimuth, targetQuality, 
                                  currentBleStatus); 
     } 
 }