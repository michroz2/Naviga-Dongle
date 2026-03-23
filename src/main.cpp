/**
 * Project: Naviga-Dongle (T-Beam v1.1 Custom E22 + Universal GPS)
 * File: main.cpp
 * Version: 1.3.3
 * Description: Рефакторинг. Выделение логики LoRa в отдельный класс RadioManager.
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <SPI.h>              
 #include <XPowersLib.h>       
 #include "configuration.h"
 #include "logger.h"           
 #include "GeoPacker.h"        
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"     
 #include "Retranslation.h"    
 #include "GpsManager.h"       
 #include "DisplayManager.h"   
 #include "RadioManager.h"     // НОВОЕ: Подключаем наш менеджер радио

 // --- НАСТРОЙКИ СКАНИРОВАНИЯ ---
 uint32_t networkScanDuration = 30000; 
 #define RED_LED_PIN 4                 

 uint8_t myNodeId = 0; 
 uint8_t myMsgSeq = 0;

 XPowersAXP2101 pmu;                                    
 DisplayManager display(0x3c, I2C_SDA, I2C_SCL); 
 GpsManager gps;                                       
 RadioManager radio; // НОВОЕ: Наш умный радиомодуль (название сохранено для совместимости)
 GeoPacker packer;                                      
 NodeDatabase nodeDB;                                   
 Retranslation router;                                  
 
 volatile bool receivedFlag = false;                    

 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
     receivedFlag = true;
 }

 float dist = 0.0;                                      
 float azmt = 0.0;                                      
 float lastSNR = 0.0;                                   
 uint8_t lastTargetId = 0;                              

 uint32_t lastTxTime = 0;                               
 uint32_t lastGpsLogTime = 0;                           
 uint32_t lastCleanupTime = 0;                          
 bool isLonScaleSet = false;                            

 int getConnectionQuality(uint8_t targetId) {
     const NodeRecord* target = nodeDB.getNode(targetId);
     if (target == nullptr || targetId == 0) return 0;
     if (millis() - target->lastSeen > 30000) return 0;

     int q = map((long)lastSNR, -20, 5, 1, 10);
     if (q < 1) q = 1;
     if (q > 10) q = 10;
     return q;
 } 

 void updateScreenCb(String line1, String line2, String line3, String line4) {
     display.showStatus(line1, line2, line3, line4);
 }

 void cycleGpsPower() {
     pmu.disableALDO3(); 
     delay(2000); 
     pmu.enableALDO3(); 
     delay(2000); 
 }

// --- ДИСПЕТЧЕР СООБЩЕНИЙ ---
void handleCoordsPacket(uint8_t senderId, const uint8_t* payload) {
    uint32_t packedCoords;
    memcpy(&packedCoords, payload, sizeof(PayloadCoords));

    if (!gps.isValid()) {
        LOG_WARN("DISPATCH", "No GPS fix. Saved RAW coords for Node %d", senderId);
        nodeDB.updateNodeCoords(senderId, 0.0f, 0.0f, packedCoords);
        lastTargetId = senderId; 
        return;
    }

    float unpLat, unpLon;
    packer.unpack(packedCoords, gps.getLat(), gps.getLon(), unpLat, unpLon);
    
    nodeDB.updateNodeCoords(senderId, unpLat, unpLon, packedCoords);
    lastTargetId = senderId; 
    LOG_INFO("DISPATCH", "Extracted COORDS: Node %d -> Lat: %.6f, Lon: %.6f", senderId, unpLat, unpLon);
}

void handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload) {
    PayloadNodeInfo info;
    memcpy(&info, payload, sizeof(PayloadNodeInfo));
    nodeDB.updateNodeName(senderId, info.nodeName);
    LOG_INFO("DISPATCH", "Updated Name: Node %d -> %s", senderId, info.nodeName);
} 

void handleLeavePacket(uint8_t senderId, const uint8_t* payload) {
    PayloadLeave info;
    memcpy(&info, payload, sizeof(PayloadLeave)); 
    nodeDB.removeNode(senderId);
    LOG_INFO("DISPATCH", "Node %d left the network. Reason code: %d", senderId, info.reason);
} 

void processIncomingPacket(const NavigaHeader& header, const uint8_t* payload) {
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

 // --- ФУНКЦИЯ ПЕРЕДАЧИ КООРДИНАТ ---
 void sendLocation() {
     if (!gps.isValid()) {
         LOG_WARN("TX", "Skip TX: GPS location not valid.");
         return;
     } 
     uint32_t packedCoords = packer.pack(gps.getLat(), gps.getLon());
     
     NavigaHeader txHeader;
     txHeader.senderId = myNodeId;
     txHeader.relayId = myNodeId; 
     txHeader.msgSeq = myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_COORDS, 15); 
     
     uint8_t txBuffer[8];
     memcpy(txBuffer, &txHeader, sizeof(NavigaHeader));
     memcpy(txBuffer + 4, &packedCoords, sizeof(packedCoords));
     
     LOG_INFO("TX", "Starting TX... Seq: %d, Packed: 0x%08X", txHeader.msgSeq, packedCoords);
     
     radio.standby();
     radio.transmit(txBuffer, 8); 
     receivedFlag = false;
     radio.startReceive();
 } 

 // --- ОБРАБОТКА КОЛЛИЗИИ (СМЕНА ID) ---
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
     
     NavigaHeader infoHeader;
     infoHeader.senderId = myNodeId;
     infoHeader.relayId = myNodeId;
     infoHeader.msgSeq = myMsgSeq++;
     infoHeader.setTypeAndTTL(MSG_NODE_INFO, 15);
     
     PayloadNodeInfo infoPayload;
     snprintf(infoPayload.nodeName, sizeof(infoPayload.nodeName), "Node-%d", myNodeId);
     
     uint8_t txBuffer[sizeof(NavigaHeader) + sizeof(PayloadNodeInfo)];
     memcpy(txBuffer, &infoHeader, sizeof(NavigaHeader));
     memcpy(txBuffer + sizeof(NavigaHeader), &infoPayload, sizeof(PayloadNodeInfo));
     
     delay(random(100, 500)); 
     radio.standby();
     radio.transmit(txBuffer, sizeof(txBuffer));
     
     lastTxTime = millis() - txInterval + 2000; 
 }

 // --- ФУНКЦИЯ СКАНИРОВАНИЯ ЭФИРА ---
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
                             processIncomingPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));
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
     
     lastTxTime = millis(); 
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
     
     if (pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
         pmu.setALDO2Voltage(3300); pmu.enableALDO2();
         pmu.setALDO3Voltage(3300); pmu.enableALDO3();
         pmu.disableALDO4(); pmu.enableSystemVoltageMeasure();  
     }
     
     display.init(); 
     display.showLogo();
     
     gps.init(updateScreenCb, cycleGpsPower); 
     
     // НОВОЕ: Элегантный старт LoRa
     display.showStatus("System Init...", "GPS Init Done", "Init LoRa...", "");
     if (!radio.init(setFlag)) {
         display.showStatus("ERROR", "LoRa Init Failed", "Check Logs", "");
         delay(3000);
     }

     scanNetworkForUniqueId();
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
                 lastSNR = radio.getSNR();
                 
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

                     if (isOwnEcho) {
                         // do nothing
                     } 
                     else if (!router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d", rxHeader.getType(), payloadLen);
                     }
                     else if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         LOG_WARN("LORA", "Duplicate pkt Node %d Seq %d dropped.", rxHeader.senderId, rxHeader.msgSeq);
                         if (router.shouldAbortRelay(rxHeader.senderId, rxHeader.msgSeq)) {
                             router.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                         }
                     }
                     else {
                         LOG_INFO("LORA", "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, TTL: %d)", 
                                  rxHeader.getType(), rxHeader.senderId, rxHeader.relayId, rxHeader.msgSeq, rxHeader.getTTL());
                         processIncomingPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));
                         
                         if (router.shouldRetransmit(rxHeader)) {
                             router.enqueuePacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, lastSNR);
                         } 
                     } 
                 } 
             } 
         } 
         radio.startReceive();
     } 

     // 3. ПЕРЕДАЧА LORA (Собственные координаты)
     if (millis() - lastTxTime >= txInterval) {
         if (!receivedFlag) { sendLocation(); lastTxTime = millis(); } 
     } 

     // 3.1. РЕТРАНСЛЯЦИЯ ПАКЕТОВ ИЗ ОЧЕРЕДИ
     NavigaHeader relayHeader;
     uint8_t relayPayload[MAX_PAYLOAD_SIZE];
     size_t relayPayloadLen;
     
     if (!receivedFlag && router.getReadyPacket(myNodeId, relayHeader, relayPayload, relayPayloadLen)) {
         uint8_t txBuffer[sizeof(NavigaHeader) + MAX_PAYLOAD_SIZE];
         size_t totalLen = sizeof(NavigaHeader) + relayPayloadLen;
         
         memcpy(txBuffer, &relayHeader, sizeof(NavigaHeader));
         memcpy(txBuffer + sizeof(NavigaHeader), relayPayload, relayPayloadLen);
         
         LOG_INFO("RELAY", "Transmitting pkt Seq %d from Node %d. New TTL: %d", 
                  relayHeader.msgSeq, relayHeader.senderId, relayHeader.getTTL());
         
         radio.standby();
         radio.transmit(txBuffer, totalLen); 
         receivedFlag = false;
         radio.startReceive();
     }

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
 
         const NodeRecord* targetNode = nodeDB.getNode(lastTargetId);
         bool isTargetValid = (targetNode != nullptr);

         if (!isTargetValid && lastTargetId != 0) lastTargetId = 0; 

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
                 if (node != nullptr && node->packedCoords != 0 && node->lat == 0.0f && node->lon == 0.0f) {
                     float unpLat, unpLon;
                     packer.unpack(node->packedCoords, gps.getLat(), gps.getLon(), unpLat, unpLon);
                     nodeDB.updateNodeCoords(i, unpLat, unpLon, node->packedCoords, false);
                 }
             }

             if (isTargetValid) {
                 dist = gps.distanceTo(targetNode->lat, targetNode->lon);
                 azmt = gps.courseTo(targetNode->lat, targetNode->lon);
             } 
         } 
 
         line2 = "My: " + String(myNodeId) + "-" + String(myMsgSeq);
         
         uint8_t totalNodes = nodeDB.getActiveNodesCount();
         uint8_t neighbors = (totalNodes > 0) ? (totalNodes - 1) : 0;
         line3 = "Neighbors: " + String(neighbors);
         
         if (isTargetValid) {
             line4 = String(targetNode->nodeId) + ": " + String((int)dist) + "m/" + String((int)azmt) + "/" + String(getConnectionQuality(targetNode->nodeId));
         } else {
             line4 = "No targets";
         } 

         display.showStatus(line1, line2, line3, line4); 
     } // if displayInterval
 } // loop()