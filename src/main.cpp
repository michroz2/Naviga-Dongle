/**
 * Project: Naviga-Dongle (T-Beam v1.1 Custom E22 + Universal GPS)
 * File: main.cpp
 * Version: 1.3.9
 * Description: Завершена архитектура единой очереди передач (Unified Tx Queue). 
 * Интегрирован алгоритм CSMA/CA с поддержкой подавления широковещания.
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
     
     LOG_WARN("COLLISION", "ID %d is taken! Switched to new ID: %d", oldId, myNodeId);
     
     myMsgSeq = 0; 
     
     if (gps.isValid()) {
         nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0, false);
     }
     
     char myName[16];
     snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
     txManager.sendNodeInfo(myName, TX_CRITICAL);
     
     lastTxTime = millis() - txInterval + 2000; 
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
     
     char myName[16];
     snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
     txManager.sendNodeInfo(myName, TX_NORMAL);
     
     lastTxTime = millis(); 
 } 
 
 void loop() {
     // 1. ЧТЕНИЕ GPS 
     gps.update();

     // 2. ОБРАБОТКА LORA ПРИЕМА
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
                     } 
                     else if (rxHeader.senderId == myNodeId) {
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
                     } 
                     else if (!router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d", rxHeader.getType(), payloadLen);
                     }
                     else if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         LOG_WARN("LORA", "Duplicate pkt Node %d Seq %d dropped.", rxHeader.senderId, rxHeader.msgSeq);
                         // НОВОЕ: Транзитный дубликат означает, что кто-то уже сделал работу за нас!
                         // Вызываем отмену ретрансляции прямо здесь.
                         txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                     }
                     else {
                         LOG_INFO("LORA", "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, SNR: %.1f)", 
                                  rxHeader.getType(), rxHeader.senderId, rxHeader.relayId, rxHeader.msgSeq, currentSNR);
                         
                         packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));
                         
                         if (router.shouldRetransmit(rxHeader)) {
                             // Передаем в очередь MAC-уровня на отправку с SNR-задержкой
                             txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, currentSNR);
                         } 
                     } 
                 } 
             } 
         } 
         radio.startReceive();
     } 

     // 3. ПЕРЕДАЧА LORA (Добавление собственных пакетов в общую очередь)
     if (millis() - lastTxTime >= txInterval) {
         if (gps.isValid()) {
             txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
         } else {
             LOG_WARN("TX", "Skip TX: GPS location not valid.");
         }
         lastTxTime = millis(); 
     } 

     // 3.1. ПРОЦЕССИНГ ЕДИНОЙ ОЧЕРЕДИ (Заменил старый ручной блок)
     txManager.processQueue();

     // 3.2. ПЕРИОДИЧЕСКАЯ ОЧИСТКА БАЗЫ 
     if (millis() - lastCleanupTime >= NODE_TIMEOUT_MS) {
         lastCleanupTime = millis();
         nodeDB.cleanup();
     } 

     // 4. ОБНОВЛЕНИЕ ДИСПЛЕЯ
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
     } // if displayInterval
 } // loop()