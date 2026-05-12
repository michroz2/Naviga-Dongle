/**
 * Project: Naviga-Dongle (T-Beam v1.1 / T-Energy S3 + Custom E22 + GPS)
 * File: main.cpp
 * Version: 1.45 (Refactored)
 * Изменение: Гео-вычисления вынесены в CoordProcessor.
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
 #include "NetworkManager.h"    
 #include "CoordProcessor.h"    // ИЗМЕНЕНИЕ 1.45
 
 // --- НАСТРОЙКИ СКАНИРОВАНИЯ ---
 uint32_t networkScanDuration = 30000; 
 
 uint8_t myNodeId = 0;   
 uint8_t myMsgSeq = 0;   
 uint8_t myNodeType = NODE_RELAY; 
 
 // Инициализация глобальных менеджеров подсистем
 PowerManager power;                                                        
 DisplayManager display(0x3c, I2C_SDA, I2C_SCL); 
 GpsManager gps;                                                        
 RadioManager radio; 
 GeoPacker packer;                                                       
 NodeDatabase nodeDB;                                                   
 Retranslation router;                                                   
 BleManager bleManager; 
 PacketManager packetManager(nodeDB, gps, packer, bleManager);
 TxManager txManager(radio, packer, myNodeId, myMsgSeq);
 NetworkManager networkManager(radio, nodeDB, display, gps, txManager, bleManager, router, packetManager);
 // ИЗМЕНЕНИЕ 1.45: Инициализация гео-процессора
 CoordProcessor coordProcessor;
 
 volatile bool receivedFlag = false;                       
 
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR 
 #endif
 void setFlag(void) {
     receivedFlag = true;
 } 
 
 // Системные таймеры
 uint32_t lastTxTime = 0;                                
 uint32_t lastGpsLogTime = 0;                            
 uint32_t lastCleanupTime = 0;                                
 uint32_t lastHeartbeatTime = 0;   
 uint32_t lastTopologyUpdateTime = 0;
 uint32_t lastTelemetryTime = 0; 
 
 // Коллбэк для обновления экрана
 void updateScreenCb(String line1, String line2, String line3, String line4) {
     display.showStatus(line1, line2, line3, line4);
 } 
 
 // Коллбэк сброса питания GPS
 void cycleGpsPowerCb() {
     power.cycleGpsPower();
 } 
 
 // === НАЧАЛЬНАЯ НАСТРОЙКА ===
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
     nodeDB.ageAllNodes(settingsManager.settings.nodeConnectionTimeout + 1000);
 
     char myName[24]; 
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
 
     if (myNodeId == 0 || !settingsManager.settings.isConfigured) {
         networkManager.scanNetwork(false, networkScanDuration, myNodeId); 
         settingsManager.settings.nodeId = myNodeId;
         settingsManager.settings.isConfigured = true;
         settingsManager.save();
     } else {
         networkManager.scanNetwork(true, networkScanDuration, myNodeId);  
     }
     
     txManager.sendNodeInfo(myName, myNodeType, TX_NORMAL);
     lastTxTime = millis(); 
 } 
 
 // === ГЛАВНЫЙ ЦИКЛ ОРКЕСТРАТОРА ===
 void loop() {
     uint32_t currentMillis = millis();
     float currentSpeed = gps.getSpeed();
 
    // ==========================================================
    // ОБРАБОТКА КОМАНД ОТ СМАРТФОНА ПО BLUETOOTH
    // ==========================================================
    
    if (bleManager.requestIdentitySync) {
        bleManager.requestIdentitySync = false;
        bleManager.sendIdentity(settingsManager.settings.nodeId, settingsManager.settings.nodeName, settingsManager.settings.nodeType);
    }

    if (bleManager.requestSysConfigSync) {
        bleManager.requestSysConfigSync = false;
        bleManager.sendSysConfig(settingsManager.settings.txIntervalMoving, settingsManager.settings.txIntervalStill, 
                                 settingsManager.settings.nodeConnectionTimeout, settingsManager.settings.nodeActiveTimeoutMs);
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
                 update.snr = node->snr;
                 update.lastSeenAge = millis() - node->lastSeen;
                 bleManager.sendNodeUpdate(update);
                 delay(5); 
             }
         }
     }
 
     if (bleManager.hasNewIdentity) {
         bleManager.hasNewIdentity = false;
         myNodeId = bleManager.newIdentity.myNodeId;
         myNodeType = bleManager.newIdentity.myRole;
         settingsManager.settings.nodeId = myNodeId;
         settingsManager.settings.nodeType = myNodeType;
         strncpy(settingsManager.settings.nodeName, bleManager.newIdentity.myName, sizeof(settingsManager.settings.nodeName)-1);
         settingsManager.save();
         txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType, TX_CRITICAL);
         nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName, myNodeType);
     }
 
     if (bleManager.hasNewSysConfig) {
        bleManager.hasNewSysConfig = false;
        settingsManager.settings.txIntervalMoving = bleManager.newSysConfig.txIntervalMoving;
        settingsManager.settings.txIntervalStill = bleManager.newSysConfig.txIntervalStill;
        settingsManager.settings.nodeConnectionTimeout = bleManager.newSysConfig.nodeConnectionTimeout;
        settingsManager.settings.nodeActiveTimeoutMs = bleManager.newSysConfig.nodeActiveTimeoutMs;
        settingsManager.save();
    }

     if (bleManager.requestClearDB) {
         bleManager.requestClearDB = false;
         for (int i = 1; i < 255; i++) { if (i != myNodeId) nodeDB.removeNode(i); }
         settingsManager.saveNodesSnapshot(nodeDB); 
     }
 
     if (bleManager.requestReset) {
         settingsManager.factoryReset();
         delay(500);
         ESP.restart();
     }
 
    // Определяем, является ли Трекер "бегущим"
    bool isFastTracker = (myNodeType == NODE_TRACKER && currentSpeed > TRACKER_FAST_SPEED_KMPH);
 
    // Периодический пересчет топологии сети
    if (gps.isValid() && (currentMillis - lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS)) {
        if (!isFastTracker) nodeDB.updateTopology();
        lastTopologyUpdateTime = currentMillis;
    }
 
    // Очистка устаревших узлов
    if (currentMillis - lastCleanupTime > CLEANUP_INTERVAL_MS) {
        nodeDB.cleanup(myNodeId); 
        lastCleanupTime = currentMillis;
    } 
 
    // Heartbeat (NodeInfo)
    if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
        if (myNodeId != 0) { 
            txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType, TX_NORMAL);
            nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName, myNodeType); 
        } 
        lastHeartbeatTime = currentMillis;
    } 
 
     gps.update();

     if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
         lastTelemetryTime = currentMillis;
         if (bleManager.getBleStatus() == BLE_CONNECTED) {
             bleManager.sendMyStatus(gps.isValid(), gps.getSatellites(), power.getBatteryPercent(), power.getBatteryVoltage());
         }
     }
 
     // === ОБРАБОТКА ВХОДЯЩИХ ПАКЕТОВ LORA (RX) ===
     if (receivedFlag) {
         noInterrupts(); receivedFlag = false; interrupts(); 
         size_t len = radio.getPacketLength();
         if (len >= sizeof(NavigaHeader)) {
             uint8_t rxBuffer[256];             
             if (radio.readData(rxBuffer, len) == RADIOLIB_ERR_NONE) {
                 float currentSNR = radio.getSNR();
                 NavigaHeader rxHeader;
                 memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                 size_t payloadLen = len - sizeof(NavigaHeader);
                 
                 if (rxHeader.relayId == myNodeId) networkManager.handleCollision(myNodeId, myMsgSeq, myNodeType);
                 else if (rxHeader.senderId == myNodeId) {
                     int8_t seqDiff = (int8_t)(myMsgSeq - rxHeader.msgSeq);
                     if (seqDiff <= 0 || seqDiff > 10) networkManager.handleCollision(myNodeId, myMsgSeq, myNodeType);
                 } 
 
                 if (rxHeader.relayId != myNodeId) nodeDB.updateNodeSNR(rxHeader.relayId, currentSNR);
 
                 if (router.isValidPacket(rxHeader.getType(), payloadLen)) {
                     if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         if (!nodeDB.hasNodesInOppositeDirection(rxHeader.relayId)) txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                     } else {
                            bool isNewNode = !nodeDB.isNodeActive(rxHeader.senderId);
                            packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen);
                            const NodeRecord* updatedNode = nodeDB.getNode(rxHeader.senderId);
                            if (updatedNode != nullptr) {
                                BleEvtNodeUpdate update;
                                update.opCode = EVT_NODE_UPDATE; update.nodeId = updatedNode->nodeId;
                                update.nodeRole = updatedNode->type; strncpy(update.nodeName, updatedNode->nodeName, 23);
                                update.lat = updatedNode->lat; update.lon = updatedNode->lon;
                                update.snr = updatedNode->snr; update.lastSeenAge = millis() - updatedNode->lastSeen;
                                bleManager.sendNodeUpdate(update);
                            }
                            if (isNewNode && rxHeader.senderId != myNodeId) {
                                lastHeartbeatTime = millis() - HEARTBEAT_INTERVAL_MS + random(MIN_GREETING_NODEINFO_JITTER, MAX_GREETING_NODEINFO_JITTER);
                                settingsManager.saveNodesSnapshot(nodeDB);
                            } 
                            if (router.shouldRetransmit(rxHeader, nodeDB, myNodeType, currentSpeed)) {
                                uint8_t sRole = NODE_STALKER;
                                const NodeRecord* sn = nodeDB.getNode(rxHeader.senderId);
                                if (sn) sRole = sn->type;
                                uint32_t jitter = networkManager.calculateRelayJitter(myNodeType, sRole, currentSNR);
                                txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, jitter);
                            } 
                     }
                 }
             }
         } 
         radio.startReceive(); 
     } 
 
    // === АДАПТИВНАЯ ОТПРАВКА КООРДИНАТ (SMART TX) ===
    if (gps.isValid()) {
        const NodeRecord* myRecord = nodeDB.getNode(myNodeId);
        float distFromLastTx = (myRecord != nullptr) ? myRecord->distance : 0.0f;
        uint32_t now = millis();
 
        bool shouldTransmit = (distFromLastTx > MIN_MOVEMENT_METERS && currentSpeed > MIN_SPEED_KMPH) || (distFromLastTx > SNEAK_MOVEMENT_METERS);
 
        if ((shouldTransmit && (now - lastTxTime >= settingsManager.settings.txIntervalMoving)) || (now - lastTxTime >= settingsManager.settings.txIntervalStill)) {
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0); 
            if (bleManager.getBleStatus() == BLE_CONNECTED) {
                BleEvtNodeUpdate u; u.opCode = EVT_NODE_UPDATE; u.nodeId = myNodeId; u.nodeRole = myNodeType;
                strncpy(u.nodeName, settingsManager.settings.nodeName, 23); u.lat = gps.getLat();
                u.lon = gps.getLon(); u.snr = 0.0f; u.lastSeenAge = 0;
                bleManager.sendNodeUpdate(u);
            }
            lastTxTime = now;
        }
    } 
 
     txManager.processQueue();
 
     // === ОБНОВЛЕНИЕ ЭКРАНА И ФОНОВЫХ РАСЧЕТОВ ===
     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         display.toggleLed();
         int sats = gps.getSatellites();
         uint8_t currentTargetId = packetManager.getLastTargetId();
         const NodeRecord* targetNode = nodeDB.getNode(currentTargetId);
         bool isTargetValid = (targetNode != nullptr && targetNode->isActive);
         if (!isTargetValid && currentTargetId != 0) { packetManager.clearLastTargetId(); currentTargetId = 0; } 
 
         // ИЗМЕНЕНИЕ 1.45: Вызов фоновых расчетов через CoordProcessor
         coordProcessor.process(nodeDB, gps, packer, isFastTracker);
 
         int tDist = isTargetValid ? (int)targetNode->distance : 0;
         int tAz = isTargetValid ? (int)targetNode->azimuth : 0;
         int tQual = isTargetValid ? networkManager.getConnectionQuality(targetNode->nodeId, myNodeId) : 0;
         display.updateMainScreen(bleManager.macSuffix, gps.isValid(), sats, myNodeId, myMsgSeq, 
                                  nodeDB.getActiveNodesCount(), isTargetValid, currentTargetId, tDist, tAz, tQual, bleManager.getBleStatus());
     } 
 }