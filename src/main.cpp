/**
 * Project: Naviga-Dongle (T-Beam v1.1 Custom E22 + Universal GPS)
 * File: main.cpp
 * Version: 1.2.7
 * Description: Система логирования GPS координат, умная инициализация GPS
 * и обмен данными через LoRa (SX1268). Основной файл программы.
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <SPI.h>              
 #include <RadioLib.h>         
 #include <XPowersLib.h>       
 #include <TinyGPS++.h>
 #include "SSD1306Wire.h"
 #include "configuration.h"
 #include "logger.h"           
 #include "GeoPacker.h"        
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"     
 #include "Retranslation.h"    

 uint8_t myNodeId = 0; 
 uint8_t myMsgSeq = 0;

 XPowersAXP2101 pmu;                                    
 SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);           
 TinyGPSPlus gps;                                       
 HardwareSerial GPS_Serial(1);                          
 GeoPacker packer;                                      
 NodeDatabase nodeDB;                                   
 Retranslation router;                                  
 
 SX1268 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
 
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

 void showStatus(String line1, String line2, String line3, String line4) {
     display.clear();
     display.setFont(ArialMT_Plain_16);
     display.drawString(0, 0,  line1);
     display.drawString(0, 16, line2);
     display.drawString(0, 32, line3);
     display.drawString(0, 48, line4);
     display.display();
 } 

 void showLogo() {
     display.clear();
     display.setFont(ArialMT_Plain_16);
     display.drawString(0, 0,  "Naviga-Dongle");
     display.drawString(0, 22, "System Init...");
     display.drawString(0, 44, "Please Wait");
     display.display();
     delay(2000);
 } 

 // --- GPS И LORA INIT (Свернуты для читаемости) ---
 const uint32_t baudRates[] = {9600, 115200, 38400, 57600, 19200, 4800};
 const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);
 uint32_t originalBaud = 0;
 const uint8_t UBX_FACTORY_RESET[] = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A };

 bool checkNMEA(uint32_t baud) {
     GPS_Serial.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
     unsigned long start = millis();
     char prevChar = 0;
     while (millis() - start < 1500) {
         if (GPS_Serial.available()) {
             char c = GPS_Serial.read();
             if (prevChar == '$' && (c == 'G' || c == 'P')) return true;
             prevChar = c;
         }
     }
     return false;
 }

 void initGPS() {
     showStatus("Init GPS...", "Searching module", "Wait...", "");
     bool nmeaFound = false;
     uint32_t activeBaud = 0;
     for (int i = 0; i < numBauds; i++) {
         if (checkNMEA(baudRates[i])) {
             activeBaud = baudRates[i];
             originalBaud = activeBaud; 
             nmeaFound = true;
             break;
         }
     }
     if (nmeaFound) {
         if (activeBaud == 9600) return; 
         else {
             showStatus("Init GPS...", "Switching Baud", String(activeBaud) + " -> 9600", "");
             GPS_Serial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n");
             GPS_Serial.flush();
             delay(500); 
             if (checkNMEA(9600)) return; 
             else nmeaFound = false;
         }
     }
     if (!nmeaFound) {
         showStatus("Init GPS...", "Rescue Mode!", "Wait 10 sec...", "");
         for (int i = 0; i < numBauds; i++) {
             GPS_Serial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
             delay(50);
             for(int j = 0; j < 3; j++) { GPS_Serial.write(UBX_FACTORY_RESET, sizeof(UBX_FACTORY_RESET)); GPS_Serial.flush(); delay(50); }
         }
         pmu.disableALDO3(); delay(2000); pmu.enableALDO3(); delay(2000); 
         if (checkNMEA(115200)) { originalBaud = 115200; GPS_Serial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n"); GPS_Serial.flush(); delay(500); } 
         else if (checkNMEA(9600)) { originalBaud = 9600; } 
         else { originalBaud = 0; }
         GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
     }
 } 

 void initLoRa() {
     showStatus("System Init...", "GPS Init Done", "Init LoRa...", "");
     int state = radio.begin(433.0);        
     if (state == RADIOLIB_ERR_NONE) {
         radio.setDio2AsRfSwitch(false);    
         radio.setBandwidth(125.0);         
         radio.setSpreadingFactor(9);       
         radio.setCodingRate(5);            
         radio.setSyncWord(0x2B);           
         radio.setPreambleLength(16);       
         radio.setOutputPower(22);          
         radio.setCurrentLimit(140);        
         radio.setTCXO(1.8);
         radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         radio.setRxBoostedGainMode(true);
         radio.setPacketReceivedAction(setFlag);
         radio.startReceive();
     } else {
         showStatus("ERROR", "LoRa Init Failed", "Check Logs", "");
         delay(3000);
     }
 } 

// --- ДИСПЕТЧЕР СООБЩЕНИЙ (Локальная обработка) ---
void handleCoordsPacket(uint8_t senderId, const uint8_t* payload) {
    if (!gps.location.isValid()) {
        LOG_WARN("DISPATCH", "Skip COORDS unpack: local GPS not valid");
        return;
    }
    uint32_t packedCoords;
    memcpy(&packedCoords, payload, sizeof(PayloadCoords));
    float unpLat, unpLon;
    packer.unpack(packedCoords, gps.location.lat(), gps.location.lng(), unpLat, unpLon);
    
    nodeDB.updateNodeCoords(senderId, unpLat, unpLon);
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
            LOG_WARN("DISPATCH", "Unknown packet type %d passed customs?!", header.getType());
            break;
    } 
} 

 // --- ФУНКЦИЯ ПЕРЕДАЧИ КООРДИНАТ ---
 void sendLocation() {
     if (!gps.location.isValid()) {
         LOG_WARN("TX", "Skip TX: GPS location not valid.");
         return;
     } 
     uint32_t packedCoords = packer.pack(gps.location.lat(), gps.location.lng());
     
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
     display.init(); display.flipScreenVertically();
     showLogo();
     initGPS(); initLoRa();

     randomSeed(esp_random());
     myNodeId = random(1, 255);
     LOG_INFO("SYS", "Generated Node ID: %d", myNodeId);
 } 
 
 void loop() {
     // 1. ЧТЕНИЕ GPS
     while (GPS_Serial.available() > 0) {
         gps.encode(GPS_Serial.read());
     } 

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
                     
                     // 1. Свое собственное эхо игнорируем
                     if (rxHeader.senderId == myNodeId) {
                         LOG_WARN("LORA", "Received our own packet. Ignored.");
                     } 
                     // 2. Валидация пакета по справочнику
                     else if (!router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d", rxHeader.getType(), payloadLen);
                     }
                     // 3. Проверка на дубликаты
                     else if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         LOG_WARN("LORA", "Duplicate pkt Node %d Seq %d dropped.", rxHeader.senderId, rxHeader.msgSeq);
                         
                         // Задел на будущее: проверка, нужно ли удалить пакет из очереди ретрансляции
                         if (router.shouldAbortRelay(rxHeader.senderId, rxHeader.msgSeq)) {
                             router.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                         }
                     }
                     // 4. Пакет прошел таможню: обрабатываем локально!
                     else {
                         LOG_INFO("LORA", "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, TTL: %d)", 
                                  rxHeader.getType(), rxHeader.senderId, rxHeader.relayId, rxHeader.msgSeq, rxHeader.getTTL());
                         
                         // Передаем полезную нагрузку в диспетчер
                         processIncomingPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));
                         
                         // 5. Оценка возможности ретрансляции и постановка в очередь
                         if (router.shouldRetransmit(rxHeader)) {
                             // Передаем SNR для расчета умной задержки
                             router.enqueuePacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, lastSNR);
                         } else {
                             LOG_INFO("LORA", "Packet reached max hops (TTL <= 1). No relay.");
                         }
                     } 
                 } else {
                     LOG_WARN("LORA", "Packet too short! Length: %d", len);
                 } 
             } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                 LOG_WARN("LORA", "CRC Error!");
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
     
     // Если эфир свободен и пришло время передавать чужой пакет
     if (!receivedFlag && router.getReadyPacket(myNodeId, relayHeader, relayPayload, relayPayloadLen)) {
         uint8_t txBuffer[sizeof(NavigaHeader) + MAX_PAYLOAD_SIZE];
         size_t totalLen = sizeof(NavigaHeader) + relayPayloadLen;
         
         // Собираем модифицированный пакет обратно
         memcpy(txBuffer, &relayHeader, sizeof(NavigaHeader));
         memcpy(txBuffer + sizeof(NavigaHeader), relayPayload, relayPayloadLen);
         
         LOG_INFO("RELAY", "Transmitting pkt Seq %d from Node %d. New TTL: %d", 
                  relayHeader.msgSeq, relayHeader.senderId, relayHeader.getTTL());
         
         radio.standby();
         radio.transmit(txBuffer, totalLen); 
         receivedFlag = false;
         radio.startReceive();
     }

     // 3.2. ПЕРИОДИЧЕСКАЯ ОЧИСТКА БАЗЫ (Раз в 5 минут)
     if (millis() - lastCleanupTime >= NODE_TIMEOUT_MS) {
         lastCleanupTime = millis();
         nodeDB.cleanup();
         LOG_INFO("SYS", "Node database cleanup performed.");
     } 

     // 4. ОБНОВЛЕНИЕ ДИСПЛЕЯ
     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
         
         String line1, line2, line3, line4;
         int sats = gps.satellites.value();
 
         const NodeRecord* targetNode = nodeDB.getNode(lastTargetId);
         bool isTargetValid = (targetNode != nullptr);

         if (!isTargetValid && lastTargetId != 0) {
             LOG_WARN("LORA", "Target Node %d is offline or timed out.", lastTargetId);
             lastTargetId = 0; 
         } 

         if (!gps.location.isValid()) {
             line1 = (sats > 0) ? ("GPS Wait " + String(sats)) : "GPS ERROR";
         } else {
             line1 = "GPS OK " + String(sats);

             nodeDB.updateNodeCoords(myNodeId, gps.location.lat(), gps.location.lng());
             
             if (!isLonScaleSet) {
                 packer.updateLonScale(gps.location.lat());
                 isLonScaleSet = true;
             } 

             if (isTargetValid) {
                 dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), targetNode->lat, targetNode->lon);
                 azmt = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), targetNode->lat, targetNode->lon);
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

         showStatus(line1, line2, line3, line4);
     } // if displayInterval
 } // loop()