/**
 * Project: Naviga-Dongle (T-Beam v1.1 Custom E22 + Universal GPS)
 * File: main.cpp
 * Version: 1.1
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
 #include "GeoPacker.h"        // Подключаем класс компактной упаковки координат
 
 // --- ОБЪЕКТЫ УПРАВЛЕНИЯ ПЕРИФЕРИЕЙ ---
 XPowersAXP2101 pmu;                                    // Объект для управления контроллером питания (PMU)
 SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);           // Объект управления OLED дисплеем по шине I2C (адрес 0x3C)
 TinyGPSPlus gps;                                       // Объект парсера NMEA строк для обработки данных GPS
 HardwareSerial GPS_Serial(1);                          // Объпаратный UART порт (UART1) для связи с GPS модулем
 GeoPacker packer;                                      // Объект для упаковки/распаковки координат в 4 байта
 
 // Инициализация радиомодуля SX1268 с указанием пинов управления из configuration.h
 SX1268 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
 
 // --- ПЕРЕМЕННЫЕ И ОБРАБОТЧИКИ ПРЕРЫВАНИЙ ---
 volatile bool receivedFlag = false;                    // Флаг, устанавливаемый в прерывании при успешном получении пакета LoRa

// Макрос для размещения функции прерывания в быстрой памяти (IRAM) для плат ESP
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 /**
  * @brief Обработчик аппаратного прерывания от радиомодуля.
  * Вызывается автоматически при срабатывании пина DIO1 (завершение приема пакета).
  */
 void setFlag(void) {
     receivedFlag = true;
 } // setFlag()

 // Глобальные переменные навигационных данных
 float dist = 0.0;                                      // Дистанция (в метрах) от текущей точки до удаленного объекта
 float azmt = 0.0;                                      // Азимут (в градусах) от текущей точки на удаленный объект
 float remoteLat = 0.0;                                 // Широта, полученная по радиоканалу
 float remoteLon = 0.0;                                 // Долгота, полученная по радиоканалу
 float lastSNR = 0.0;                                   // SNR (Отношение сигнал/шум) последнего принятого пакета

 // --- ТАЙМЕРЫ И ФЛАГИ СОСТОЯНИЙ ---
 uint32_t lastTxTime = 0;                               // Время (в мс) последней отправки координат в эфир
 uint32_t lastGpsLogTime = 0;                           // Время (в мс) последнего обновления дисплея и логов
 uint32_t lastRxTime = 0;                               // Время (в мс) последнего успешного приема чужих координат
 bool isTimeoutLogged = false;                          // Флаг для предотвращения спама в лог при таймауте
 bool isLonScaleSet = false;                            // Флаг единоразовой установки множителя долготы

 // --- ФУНКЦИИ ИНТЕРФЕЙСА ---

 /**
  * @brief Рассчитывает качество радиосвязи от 0 до 10.
  * 0 - нет связи (таймаут).
  * 1..10 - вычисляется на основе SNR (Signal-to-Noise Ratio).
  */
 int getConnectionQuality() {
     // Если пакетов не было больше 30 секунд или вообще не было
     if (lastRxTime == 0 || (millis() - lastRxTime > 30000)) {
         return 0;
     } // if (timeout)

     // LoRa может демодулировать сигнал при SNR до -20 дБ. Отличным считается SNR > +5 дБ.
     int q = map((long)lastSNR, -20, 5, 1, 10);
     
     if (q < 1) q = 1;
     if (q > 10) q = 10;
     
     return q;
 } // getConnectionQuality()

 /**
  * @brief Выводит стандартизированное 4-строчное текстовое меню на OLED дисплей.
  */
 void showStatus(String line1, String line2, String line3, String line4) {
     display.clear();
     display.setFont(ArialMT_Plain_16);
     display.drawString(0, 0,  line1);
     display.drawString(0, 16, line2);
     display.drawString(0, 32, line3);
     display.drawString(0, 48, line4);
     display.display();
 } // showStatus()

 /**
  * @brief Отображает стартовый логотип и системное сообщение при включении устройства.
  */
 void showLogo() {
     display.clear();
     display.setFont(ArialMT_Plain_16);
     display.drawString(0, 0,  "Naviga-Dongle");
     display.drawString(0, 22, "System Init...");
     display.drawString(0, 44, "Please Wait");
     display.display();
     delay(2000); // Держим логотип на экране 2 секунды
 } // showLogo()

 // --- GPS INIT LOGIC (Автопоиск и настройка) ---
 const uint32_t baudRates[] = {9600, 115200, 38400, 57600, 19200, 4800};
 const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);
 uint32_t originalBaud = 0;

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
             } // if marker
             prevChar = c;
         } // if avail
     } // while
     LOG_WARN("GPS", "Fail");
     return false;
 } // checkNMEA()

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
         } // if found
     } // for

     if (nmeaFound) {
         if (activeBaud == 9600) {
             LOG_INFO("GPS", "GPS is ready at 9600 baud.");
             return; 
         } else {
             showStatus("Init GPS...", "Switching Baud", String(activeBaud) + " -> 9600", "");
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
             } // check
         } // if 9600
     } // if found

     if (!nmeaFound) {
         showStatus("Init GPS...", "Rescue Mode!", "Wait 10 sec...", "");
         LOG_INFO("GPS", "ENTERING GPS RESCUE MODE");
         for (int i = 0; i < numBauds; i++) {
             GPS_Serial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
             delay(50);
             for(int j = 0; j < 3; j++) {
                 GPS_Serial.write(UBX_FACTORY_RESET, sizeof(UBX_FACTORY_RESET));
                 GPS_Serial.flush();
                 delay(50);
             } // for j
         } // for i
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
         } // check results
         GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
         LOG_INFO("GPS", "GPS Init Protocol Finished. Listening at 9600.");
     } // if not found
 } // initGPS()

 // --- LORA INIT LOGIC ---
 void initLoRa() {
     showStatus("System Init...", "GPS Init Done", "Init LoRa...", "");
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
         } // if tcxo
         radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         state = radio.setRxBoostedGainMode(true);
         if (state != RADIOLIB_ERR_NONE) {
             LOG_ERROR("LORA", "RX Boost setup failed!");
         } // if boost
         radio.setPacketReceivedAction(setFlag);
         state = radio.startReceive();
         if (state == RADIOLIB_ERR_NONE) {
             LOG_INFO("LORA", "Reception STARTED");
         } else {
             LOG_ERROR("LORA", "Failed to start reception, code: %d", state);
         } // start rx
     } else {
         LOG_ERROR("LORA", "Radio init failed, code: %d", state);
         showStatus("ERROR", "LoRa Init Failed", "Check Logs", "");
         delay(3000);
     } // if state
 } // initLoRa()

 // --- ФУНКЦИЯ ПЕРЕДАЧИ КООРДИНАТ ---
 void sendLocation() {
     if (!gps.location.isValid()) {
         LOG_WARN("TX", "Skip TX: GPS location not valid.");
         return;
     } // if invalid

     // Упаковываем координаты в компактные 4 байта (uint32_t)
     uint32_t packedCoords = packer.pack(gps.location.lat(), gps.location.lng());
     
     uint8_t txBuffer[4];
     memcpy(txBuffer, &packedCoords, 4);
     
     LOG_INFO("TX", "Starting transmission... (Packed: 0x%08X)", packedCoords);
     
     radio.standby();
     int state = radio.transmit(txBuffer, 4); // Передаем теперь 4 байта
     if (state == RADIOLIB_ERR_NONE) {
         LOG_INFO("TX", "Transmission finished successfully.");
     } else {
         LOG_ERROR("TX", "Transmission failed, code: %d", state);
     } // if state
     
     receivedFlag = false;
     radio.startReceive();
 } // sendLocation()
 
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
     LOG_INFO("PMU", "Initializing AXP2101...");
     bool pmuFound = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
     if (pmuFound) {
         pmu.setALDO2Voltage(3300);         
         pmu.enableALDO2();
         pmu.setALDO3Voltage(3300);         
         pmu.enableALDO3();
         pmu.disableALDO4();                
         pmu.enableSystemVoltageMeasure();  
     } // if pmu
     display.init();
     display.flipScreenVertically();
     showLogo();
     initGPS();
     initLoRa();
     LOG_INFO("SYS", "Setup completed successfully! System is READY.");
 } // setup()
 
 void loop() {
     // 1. ЧТЕНИЕ GPS
     while (GPS_Serial.available() > 0) {
         gps.encode(GPS_Serial.read());
     } // while

     // 2. ОБРАБОТКА LORA ПРИЕМА
     if (receivedFlag) {
         noInterrupts();                    
         receivedFlag = false;
         interrupts();
         size_t len = radio.getPacketLength();
         if (len > 0) {
             uint8_t rxBuffer[256];             
             int state = radio.readData(rxBuffer, len); 
             if (state == RADIOLIB_ERR_NONE) {
                 LOG_INFO("LORA", "Packet Received! Length: %d bytes", len);
                 lastSNR = radio.getSNR();
                 
                 // Если длина совпадает с ожидаемой (теперь 4 байта = GeoPacker)
                 if (len == 4) {
                     // Мы можем распаковать координаты ТОЛЬКО если у нас есть свои опорные координаты
                     if (gps.location.isValid()) {
                         uint32_t packedCoords;
                         memcpy(&packedCoords, rxBuffer, 4);
                         
                         // Распаковываем координаты относительно нашей позиции
                         packer.unpack(packedCoords, gps.location.lat(), gps.location.lng(), remoteLat, remoteLon);
                         
                         lastRxTime = millis();
                         isTimeoutLogged = false; // Сбрасываем флаг, так как связь восстановлена
                         LOG_INFO("LORA", "Remote Location Extracted: Lat: %.6f, Lon: %.6f", remoteLat, remoteLon);
                     } else {
                         LOG_WARN("LORA", "Packed coords received, but local GPS not valid for unpacking!");
                     } // if isValid else
                 } else {
                     String hexStr = "";
                     for (size_t i = 0; i < len; i++) {
                         char buf[4];
                         sprintf(buf, "%02X ", rxBuffer[i]);
                         hexStr += buf;
                     } // for
                     LOG_INFO("LORA", "Unknown Data (HEX): %s", hexStr.c_str());
                 } // if len 4 else
             } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                 LOG_WARN("LORA", "CRC Error!");
             } else {
                 LOG_ERROR("LORA", "Reception failed, code: %d", state);
             } // if state
         } // if len > 0
         radio.startReceive();
     } // if receivedFlag

     // 3. ПЕРЕДАЧА LORA
     if (millis() - lastTxTime >= txInterval) {
         if (!receivedFlag) {
             sendLocation();
             lastTxTime = millis();
         } // if air free
     } // if txInterval

     // 4. ОБНОВЛЕНИЕ ДИСПЛЕЯ И ЛОГИРОВАНИЕ ТАЙМАУТА
     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
         
         String gpsStatus, distStr, azmtStr;
         int sats = gps.satellites.value();
 
         // Проверка таймаута связи
         bool isRemoteValid = false;
         if (lastRxTime == 0 || (millis() - lastRxTime > 30000)) {
             distStr = "Dist: ???";
             azmtStr = "Azmt: ???";
             
             // Логируем предупреждение ОДИН РАЗ за период таймаута
             if (!isTimeoutLogged) {
                 LOG_WARN("LORA", "Remote signal timeout (> 30s) or not established");
                 isTimeoutLogged = true; // Блокируем повторный вывод до получения нового пакета
             } // if not logged
         } else {
             isRemoteValid = true;
         } // if timeout else
 
         // Проверка своего GPS и расчеты
         if (!gps.location.isValid()) {
             gpsStatus = (sats > 0) ? ("GPS Wait " + String(sats)) : "GPS ERROR";
             distStr = "Dist: ***";
             azmtStr = "Azmt: ***";
         } else {
             gpsStatus = "GPS OK " + String(sats);
             
             // ОДНОКРАТНАЯ ИНИЦИАЛИЗАЦИЯ МАСШТАБА ДОЛГОТЫ
             if (!isLonScaleSet) {
                 packer.updateLonScale(gps.location.lat());
                 isLonScaleSet = true;
                 LOG_INFO("GPS", "LonScale locked to: %.2f", packer.getLonScale());
             } // if not set

             // ИЗМЕНЕНИЕ ЗДЕСЬ: Вывод сырого HDOP и примерной точности в метрах
             LOG_INFO("GPS", "Fix OK! Pos: %.6f, %.6f | Alt: %.1fm | HDOP: %.1f (~%dm)", 
                      gps.location.lat(), gps.location.lng(), gps.altitude.meters(), 
                      gps.hdop.hdop(), (int)(gps.hdop.hdop() * 2.5));

             if (isRemoteValid) {
                 dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), remoteLat, remoteLon);
                 azmt = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), remoteLat, remoteLon);
                 distStr = "Dist: " + String((int)dist) + " m";
                 azmtStr = "Azmt: " + String((int)azmt) + " deg";
             } // if remote valid
         } // if gps valid else
 
         showStatus(gpsStatus, distStr, azmtStr, "Conn: " + String(getConnectionQuality()));
     } // if displayInterval
 } // loop()