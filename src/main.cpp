/**
 * Project: Naviga-Dongle (T-Beam v1.1 / T-Energy S3 + Custom E22 + GPS)
 * File: main.cpp
 * Version: 1.46.7
 * Изменение: Обновление вызовов GpsManager согласно разделению на hasFix() и hasAnchor().
 * Description: Главный файл оркестратора.
 */

 #include "BleManager.h"
 #include "DisplayManager.h"
 #include "GeoPacker.h"
 #include "GpsManager.h"
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"
 #include "PacketManager.h"
 #include "PowerManager.h"
 #include "RadioManager.h"
 #include "Retranslation.h"
 #include "SettingsManager.h"
 #include "SmartPositionManager.h"
 #include "RxManager.h"
 #include "TxManager.h"
 #include "DBManager.h"
 #include "BleCommandHandler.h"
 #include "configuration.h"
 #include "logger.h"
 #include <Arduino.h>
 #include <SPI.h>
 #include <Wire.h>
 
 uint32_t networkScanDuration = 30000; 
 
 uint8_t myNodeId = 0; 
 uint8_t myMsgSeq = 0; 
 uint8_t myNodeType = NODE_RELAY; 
 
 uint32_t lastGpsLogTime = 0;
 uint32_t lastHeartbeatTime = 0;
 uint32_t lastTelemetryTime = 0; 
 
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
 
 SmartPositionManager smartPosManager(gps, nodeDB, txManager, bleManager,
                                      settingsManager, myNodeId, myNodeType);
 
 DBManager dbManager(nodeDB, txManager, bleManager, settingsManager, gps, packer);
 
 BleCommandHandler bleCommandHandler(bleManager, dbManager, settingsManager, txManager, nodeDB, myNodeId, myNodeType);
 
 void handleCollision() {
      dbManager.handleCollision(myNodeId, myMsgSeq, myNodeType);
 }
 
 RxManager rxManager(radio, router, packetManager, nodeDB, txManager, bleManager, 
                      gps, settingsManager, myNodeId, myNodeType, myMsgSeq, 
                      lastHeartbeatTime, handleCollision);
 
 int getConnectionQuality(uint8_t targetId) {
    if (targetId == myNodeId) return 10;
    const NodeRecord *target = nodeDB.getNode(targetId);
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
 void cycleGpsPowerCb() { power.cycleGpsPower(); }
 
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
          String startTitle = "Start-" + String(bleManager.macSuffix);
          uint8_t totalNodes = nodeDB.getActiveNodesCount();
          uint8_t foundNeighbors = (totalNodes > 0) ? (totalNodes - 1) : 0;
          display.showStatus(startTitle, "Time left: " + String(left) + " s",
                             "Neighbors: " + String(foundNeighbors), "Please wait...");
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
                      packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen);
                   }
                }
             }
          }
          radio.startReceive();
       }
       gps.update(); 
    }
 
    display.toggleLed();
 
    if (!isWarmStart || myNodeId == 0) {
       myNodeId = dbManager.generateUniqueId();
       nodeDB.addNode(myNodeId);
       LOG_INFO("SYS", "Scan complete. Selected unique Node ID: %d", myNodeId);
    } else {
       LOG_INFO("SYS", "Silent listening complete. Kept Node ID: %d", myNodeId);
    }
 
    display.showStatus("Scan Complete", "My ID:", String(myNodeId), "Starting...");
    delay(2000);
 }
 
 void sendHeartbeat() {
    if (myNodeId != 0) {
       txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType, TX_NORMAL);
       nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName, myNodeType);
       LOG_INFO("ACTION", "Heartbeat sent: NodeInfo");
    }
 }
 
 void sendTelemetry() {
    if (bleManager.getBleStatus() == BLE_CONNECTED) {
       // ИЗМЕНЕНИЕ 1.46.7: В телеметрию отдаем строго состояние спутникового фикса
       uint8_t gpsValid = gps.hasFix() ? 1 : 0;
       uint8_t satellites = (uint8_t)gps.getSatellites();
       uint8_t battPct = power.getBatteryPercent();
       uint16_t battV = power.getBatteryVoltage();
       bleManager.sendMyStatus(gpsValid, satellites, battPct, battV);
    }
 }
 
 void updateDisplay() {
    int sats = gps.getSatellites();
    uint8_t currentTargetId = packetManager.getLastTargetId();
    const NodeRecord *targetNode = nodeDB.getNode(currentTargetId);
    bool isTargetValid = (targetNode != nullptr && targetNode->isActive);
 
    if (!isTargetValid && currentTargetId != 0) {
       packetManager.clearLastTargetId();
       currentTargetId = 0;
    }
 
    int targetDist = isTargetValid ? (int)targetNode->distance : 0;
    int targetAzimuth = isTargetValid ? (int)targetNode->azimuth : 0;
    int targetQuality = isTargetValid ? getConnectionQuality(targetNode->nodeId) : 0;
 
    BleStatus currentBleStatus = bleManager.getBleStatus();
 
    // ИЗМЕНЕНИЕ 1.46.7: Экран отображает статус реального фикса (OK/Wait/ERR)
    display.updateMainScreen(bleManager.macSuffix, gps.hasFix(), sats, myNodeId,
                             myMsgSeq, nodeDB.getActiveNodesCount(),
                             isTargetValid, currentTargetId, targetDist,
                             targetAzimuth, targetQuality, currentBleStatus);
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
    nodeDB.ageAllNodes(settingsManager.settings.nodeConnectionTimeout + 1000);
 
    char myName[24]; 
    strncpy(myName, settingsManager.settings.nodeName, sizeof(myName) - 1);
    myName[sizeof(myName) - 1] = '\0';
 
    nodeDB.addNode(myNodeId);
    nodeDB.updateNodeInfo(myNodeId, myName, myNodeType);
 
    // ИЗМЕНЕНИЕ 1.46.7: Инициализация дефолтной опорной точки (STATIC_LAT/STATIC_LON) из RAM
    gps.setAnchorLocation(RELAY_STATIC_LAT, RELAY_STATIC_LON);
 
    display.showStatus("System Init...", "GPS Init Done", "Init LoRa...", "");
    
    if (!radio.init(setFlag)) {
       display.showStatus("ERROR", "LoRa Init Failed", "Check Logs", "");
       delay(3000);
    }
 
    if (myNodeId == 0 || !settingsManager.settings.isConfigured) {
       scanNetwork(false); 
       settingsManager.settings.nodeId = myNodeId;
       settingsManager.settings.isConfigured = true;
       settingsManager.save();
    } else {
       scanNetwork(true); 
    }
 
    txManager.sendNodeInfo(myName, myNodeType, TX_NORMAL);
    smartPosManager.resetTimer();
 }
 
 void loop() {
    uint32_t currentMillis = millis();
    float currentSpeed = gps.getSpeed();
 
    bleCommandHandler.process();
 
    bool isFastTracker = (myNodeType == NODE_TRACKER && currentSpeed > TRACKER_FAST_SPEED_KMPH);
 
    dbManager.processBackgroundTasks(isFastTracker, myNodeId);
 
    if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
       sendHeartbeat();
       lastHeartbeatTime = currentMillis;
    }
 
    gps.update();
 
    if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
       sendTelemetry();
       lastTelemetryTime = currentMillis;
    }
 
    if (rxManager.hasNewPacket()) {
        rxManager.process();
    }
 
    smartPosManager.process();
 
    txManager.processQueue();
 
    if (currentMillis - lastGpsLogTime >= gpsUpdateInterval) {
       lastGpsLogTime = currentMillis;
       display.toggleLed();
       dbManager.updateGeodata(isFastTracker);
       updateDisplay();
    }
 }