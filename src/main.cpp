/**
 * Project: Naviga-Dongle (T-Beam v1.1 / T-Energy S3 + Custom E22 + GPS)
 * File: main.cpp
 * Version: 1.43.7
 * Изменение: Рефакторинг (Шаг 7) - Изоляция логики команд Bluetooth.
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
 
 // --- НАСТРОЙКИ СКАНИРОВАНИЯ ---
 uint32_t networkScanDuration = 30000; // 30 секунд сканирования при включении
 
 uint8_t myNodeId = 0; // Локальный ID устройства в Mesh-сети
 uint8_t myMsgSeq = 0; // Счетчик исходящих пакетов (sequence)
 uint8_t myNodeType = NODE_RELAY; // Роль узла по умолчанию (перезапишется из настроек)
 
 // Системные таймеры
 uint32_t lastGpsLogTime = 0;
 uint32_t lastHeartbeatTime = 0;
 uint32_t lastTelemetryTime = 0; 
 
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
 
 SmartPositionManager smartPosManager(gps, nodeDB, txManager, bleManager,
                                      settingsManager, myNodeId, myNodeType);
 
 DBManager dbManager(nodeDB, txManager, bleManager, settingsManager, gps, packer);
 
 // ИЗМЕНЕНИЕ 1.43.7: Инициализация нового диспетчера команд Bluetooth
 BleCommandHandler bleCommandHandler(bleManager, dbManager, settingsManager, txManager, nodeDB, myNodeId, myNodeType);
 
 // Коллбэк для коллизий (делегирует выполнение в DBManager)
 void handleCollision() {
     dbManager.handleCollision(myNodeId, myMsgSeq, myNodeType);
 }
 
 RxManager rxManager(radio, router, packetManager, nodeDB, txManager, bleManager, 
                     gps, settingsManager, myNodeId, myNodeType, myMsgSeq, 
                     lastHeartbeatTime, handleCollision);
 
 // Расчет показателя "качества связи" для вывода на экран (от 1 до 10)
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
 
 // Коллбэки
 void updateScreenCb(String line1, String line2, String line3, String line4) {
   display.showStatus(line1, line2, line3, line4);
 }
 void cycleGpsPowerCb() { power.cycleGpsPower(); }
 
 // Универсальная функция первоначального сканирования и немого периода
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
     uint8_t gpsValid = gps.isValid() ? 1 : 0;
     uint8_t sats = (uint8_t)gps.getSatellites();
     uint8_t battPct = power.getBatteryPercent();
     uint16_t battV = power.getBatteryVoltage();
     bleManager.sendMyStatus(gpsValid, sats, battPct, battV);
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
 
   display.updateMainScreen(bleManager.macSuffix, gps.isValid(), sats, myNodeId,
                            myMsgSeq, nodeDB.getActiveNodesCount(),
                            isTargetValid, currentTargetId, targetDist,
                            targetAzimuth, targetQuality, currentBleStatus);
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
   strncpy(myName, settingsManager.settings.nodeName, sizeof(myName) - 1);
   myName[sizeof(myName) - 1] = '\0';
 
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
 
 // === ГЛАВНЫЙ ЦИКЛ ОРКЕСТРАТОРА ===
 void loop() {
   uint32_t currentMillis = millis();
   float currentSpeed = gps.getSpeed();
 
   // 1. Диспетчер команд от смартфона
   bleCommandHandler.process();
 
   bool isFastTracker = (myNodeType == NODE_TRACKER && currentSpeed > TRACKER_FAST_SPEED_KMPH);
 
   // 2. Фоновые задачи Базы (Синхронизация топологии, Garbage Collector)
   dbManager.processBackgroundTasks(isFastTracker, myNodeId);
 
   // 3. Регулярная отправка "живого" пинга (NodeInfo) - Heartbeat
   if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
     sendHeartbeat();
     lastHeartbeatTime = currentMillis;
   }
 
   // 4. Обновляем данные с GPS-чипа
   gps.update();
 
   // 5. Регулярная отправка Телеметрии по Bluetooth
   if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
     sendTelemetry();
     lastTelemetryTime = currentMillis;
   }
 
   // 6. Событийная обработка эфира (Только если пришел пакет)
   if (rxManager.hasNewPacket()) {
       rxManager.process();
   }
 
   // 7. Адаптивная отправка собственных координат (Smart TX)
   smartPosManager.process();
 
   // 8. Обработка очереди на передачу в радиоэфир (CSMA/CA)
   txManager.processQueue();
 
   // 9. Ежесекундные расчеты и отрисовка UI
   if (currentMillis - lastGpsLogTime >= gpsUpdateInterval) {
     lastGpsLogTime = currentMillis;
     display.toggleLed();
     dbManager.updateGeodata(isFastTracker);
     updateDisplay();
   }
 } // MAIN.CPP