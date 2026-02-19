/**
 * Project: Naviga-Dongle (T-Beam v1.1 Custom E22 + Universal GPS)
 * File: main.cpp
 * Description: Система логирования GPS координат, умная инициализация GPS
 * и обмен данными через LoRa (SX1268).
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
 
 // --- ОБЪЕКТЫ УПРАВЛЕНИЯ ПЕРИФЕРИЕЙ ---
 XPowersAXP2101 pmu;                                    
 SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);           
 TinyGPSPlus gps;                                       
 HardwareSerial GPS_Serial(1);                          
 
 // Инициализация радиомодуля SX1268 с указанием пинов управления
 SX1268 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
 
 // --- ПЕРЕМЕННЫЕ И ОБРАБОТЧИКИ ПРЕРЫВАНИЙ ---
 volatile bool receivedFlag = false;                    

 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
     receivedFlag = true;
 }

 // Глобальные переменные навигационных данных
 float dist = 0.0;                                      
 float azmt = 0.0;                                      
 float remoteLat = 0.0;
 float remoteLon = 0.0;

 // --- ТАЙМЕРЫ УПРАВЛЕНИЯ ПОТОКАМИ ---
 uint32_t lastTxTime = 0;                               
 uint32_t lastGpsLogTime = 0;                           

 // --- GPS INIT LOGIC ---
 const uint32_t baudRates[] = {9600, 115200, 38400, 57600, 19200, 4800};
 const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);
 uint32_t originalBaud = 0;

 // Бинарная команда Factory Reset (UBX-CFG-CFG)
 const uint8_t UBX_FACTORY_RESET[] = {
     0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 
     0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
     0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A
 };

 bool checkNMEA(uint32_t baud) {
     GPS_Serial.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
     LOG_INFO("GPS", "Checking %d baud...", baud);
     
     unsigned long start = millis();
     char prevChar = 0;
     
     while (millis() - start < 1500) {
         if (GPS_Serial.available()) {
             char c = GPS_Serial.read();
             if (prevChar == '$' && (c == 'G' || c == 'P')) {
                 LOG_INFO("GPS", "OK (Valid NMEA Found!)");
                 return true;
             }
             prevChar = c;
         }
     }
     LOG_WARN("GPS", "Fail");
     return false;
 }

 void initGPS() {
     display.clear();
     display.drawString(0, 0, "Init GPS...");
     display.display();

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
         if (activeBaud == 9600) {
             LOG_INFO("GPS", "GPS is ready at 9600 baud.");
             return; 
         } else {
             display.drawString(0, 15, "Switching Baud...");
             display.display();
             LOG_INFO("GPS", "Sending $PUBX command at %d baud to switch to 9600...", activeBaud);
             
             GPS_Serial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n");
             GPS_Serial.flush();
             delay(500); 
             
             LOG_INFO("GPS", "Verifying switch to 9600...");
             if (checkNMEA(9600)) {
                 LOG_INFO("GPS", "Successfully switched to 9600!");
                 return; 
             } else {
                 LOG_WARN("GPS", "Switch verification FAILED. Escalating to Rescue Mode.");
                 nmeaFound = false;
             }
         }
     }

     if (!nmeaFound) {
         display.clear();
         display.drawString(0, 0, "Rescue Mode...");
         display.display();
         LOG_INFO("GPS", "ENTERING GPS RESCUE MODE");

         for (int i = 0; i < numBauds; i++) {
             GPS_Serial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
             delay(50);
             for(int j = 0; j < 3; j++) {
                 GPS_Serial.write(UBX_FACTORY_RESET, sizeof(UBX_FACTORY_RESET));
                 GPS_Serial.flush();
                 delay(50);
             }
         }

         LOG_INFO("GPS", "Power cycling GPS...");
         pmu.disableALDO3();
         delay(2000); 
         pmu.enableALDO3();
         delay(2000); 

         LOG_INFO("GPS", "Re-evaluating after Rescue...");
         if (checkNMEA(115200)) {
             originalBaud = 115200;
             LOG_INFO("GPS", "Rescued at 115200. Sending switch command...");
             GPS_Serial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n");
             GPS_Serial.flush();
             delay(500);
         } else if (checkNMEA(9600)) {
             originalBaud = 9600;
         } else {
             originalBaud = 0;
         }
         
         GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
         LOG_INFO("GPS", "GPS Init Protocol Finished. Listening at 9600.");
     }
 }

 // --- ФУНКЦИЯ ПЕРЕДАЧИ КООРДИНАТ ---
 void sendLocation() {
     if (!gps.location.isValid()) {
         LOG_WARN("TX", "Skip TX: GPS location not valid.");
         return;
     }

     int32_t latInt = (int32_t)(gps.location.lat() * 100000);
     int32_t lonInt = (int32_t)(gps.location.lng() * 100000);

     uint8_t txBuffer[8];
     memcpy(txBuffer, &latInt, 4);
     memcpy(txBuffer + 4, &lonInt, 4);

     LOG_INFO("TX", "Starting transmission... (Lat: %d, Lon: %d)", latInt, lonInt);

     radio.standby();
     int state = radio.transmit(txBuffer, 8);

     if (state == RADIOLIB_ERR_NONE) {
         LOG_INFO("TX", "Transmission finished successfully.");
     } else {
         LOG_ERROR("TX", "Transmission failed, code: %d", state);
     }

     radio.startReceive();
 }
 
 void showStatus(String line1, String line2, String line3) {
     display.clear();
     display.setFont(ArialMT_Plain_16);
     display.drawString(0, 0,  line1);
     display.drawString(0, 22, line2);
     display.drawString(0, 44, line3);
     display.display();
 }
 
 void setup() {
     delay(500); 
     Serial.begin(115200);
     unsigned long start = millis();
     while (!Serial && (millis() - start < 3000));
     
     LOG_INFO("SYS", "--- DONGLE BOOT START ---");
 
     // 0. ИЗОЛЯЦИЯ ВСТРОЕННОГО МОДУЛЯ
     pinMode(LORA_ONBOARD_CS, OUTPUT);
     digitalWrite(LORA_ONBOARD_CS, HIGH);
 
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF);
 
     // 1. ИНИЦИАЛИЗАЦИЯ ШИН ДАННЫХ
     Wire.begin(I2C_SDA, I2C_SCL);
     SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
 
     // 2. КОНФИГУРАЦИЯ ПИТАНИЯ (AXP2101)
     LOG_INFO("PMU", "Initializing AXP2101...");
     bool pmuFound = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
     
     if (pmuFound) {
         pmu.setALDO2Voltage(3300);         // Питание LoRa модуля VCC
         pmu.enableALDO2();
         pmu.setALDO3Voltage(3300);         // Питание GPS модуля
         pmu.enableALDO3();
         pmu.disableALDO4();                // Отключение неиспользуемых каналов
         pmu.enableSystemVoltageMeasure();
     }
 
     // 3. ПОДГОТОВКА ИНТЕРФЕЙСА
     display.init();
     display.flipScreenVertically();
     showStatus("System Init...", "Power OK", "Init GPS...");
 
     // 4. УМНАЯ ИНИЦИАЛИЗАЦИЯ GPS
     initGPS();
 
     // 5. КОНФИГУРАЦИЯ RADIOLIB (SX1268)
     showStatus("System Init...", "GPS Init Done", "Init LoRa...");
     LOG_INFO("LORA", "Initializing SX1268...");
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
 
         if (radio.setTCXO(1.8) != RADIOLIB_ERR_NONE) {
              LOG_ERROR("LORA", "TCXO setup failed!");
         }
 
         radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         
         state = radio.setRxBoostedGainMode(true);
         if (state != RADIOLIB_ERR_NONE) {
             LOG_ERROR("LORA", "RX Boost setup failed!");
         }

         radio.setPacketReceivedAction(setFlag);
         
         state = radio.startReceive();
         if (state == RADIOLIB_ERR_NONE) {
             LOG_INFO("LORA", "Reception STARTED");
         }
     }
 
     // Индикация завершения инициализации
     for(int i=0; i<3; i++) {
         digitalWrite(LED_PIN, LED_ON);
         delay(100);
         digitalWrite(LED_PIN, LED_OFF);
         delay(100);
     }
 }
 
 void loop() {
     // 1. НЕПРЕРЫВНОЕ ЧТЕНИЕ ДАННЫХ GPS
     while (GPS_Serial.available() > 0) {
         gps.encode(GPS_Serial.read());
     }

     // 2. ОБРАБОТКА ПРИНЯТЫХ ПАКЕТОВ LORA (ПО ПРЕРЫВАНИЮ)
     if (receivedFlag) {
         noInterrupts();                    
         receivedFlag = false;
         interrupts();

         size_t len = radio.getPacketLength();
         uint8_t rxBuffer[256];             
         int state = radio.readData(rxBuffer, len);

         if (state == RADIOLIB_ERR_NONE) {
             LOG_INFO("LORA", "Packet Received! Length: %d bytes", len);
             
             if (len == 8) {
                 int32_t latInt, lonInt;
                 memcpy(&latInt, rxBuffer, 4);
                 memcpy(&lonInt, rxBuffer + 4, 4);

                 remoteLat = latInt / 100000.0;
                 remoteLon = lonInt / 100000.0;

                 LOG_INFO("LORA", "Remote Location Extracted: Lat: %.6f, Lon: %.6f", remoteLat, remoteLon);
                 LOG_INFO("LORA", "RSSI: %.2f dBm | SNR: %.2f dB", radio.getRSSI(), radio.getSNR());
             } else {
                 String hexStr = "";
                 for (size_t i = 0; i < len; i++) {
                     char buf[4];
                     sprintf(buf, "%02X ", rxBuffer[i]);
                     hexStr += buf;
                 }
                 LOG_INFO("LORA", "Unknown Data (HEX): %s", hexStr.c_str());
             }
             
         } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
             LOG_WARN("LORA", "CRC Error!");
         } else {
             LOG_ERROR("LORA", "Reception failed, code: %d", state);
         }
         
         radio.startReceive();
     }

     // 3. ПЛАНИРОВЩИК ПЕРЕДАЧИ
     if (millis() - lastTxTime >= txInterval) {
         if (!receivedFlag) {
             sendLocation();
             lastTxTime = millis();
         } else {
             LOG_WARN("TX", "TX delayed: Air is busy.");
         }
     }

     // 4. ПЛАНИРОВЩИК ОБНОВЛЕНИЯ ИНТЕРФЕЙСА И ЛОГИРОВАНИЯ
     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         
         // Мигание красным LED (напоминаю, что мы ранее определили, что им управляет IO4, low level - это вшито в макросы конфигурации)
         digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
         
         String gpsStatus;
         int sats = gps.satellites.value();
 
         if (gps.location.isValid()) {
             gpsStatus = "GPS OK " + String(sats);
             LOG_INFO("GPS", "Fix OK! Pos: %.6f, %.6f | Alt: %.1fm", 
                      gps.location.lat(), gps.location.lng(), gps.altitude.meters());
         } else if (sats > 0) {
             gpsStatus = "GPS Wait " + String(sats);
         } else {
             gpsStatus = "GPS ERROR";
         }
 
         // Обновление UI
         showStatus(gpsStatus, "Dist: " + String(dist, 1), "Azmt: " + String(azmt, 1));
     }
 }