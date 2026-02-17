/**
 * Project: Dongle (T-Beam v1.1 Custom E22)
 * File: main.cpp
 * Description: Hardware Initialization + GPS Detailed Logging
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <XPowersLib.h>
 #include <TinyGPS++.h>
 #include "SSD1306Wire.h"
 #include "configuration.h"
 #include "logger.h"           // Подключаем наш новый логгер
 
 // --- ОБЪЕКТЫ ---
 XPowersAXP2101 pmu;              // Используем конкретный класс AXP2101
 SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);
 TinyGPSPlus gps;
 HardwareSerial GPS_Serial(1);
 
 // --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---
 void showStatus(String line1, String line2) {
     display.clear();
     display.drawString(0, 0, "DONGLE v1.0");
     display.drawString(0, 20, line1);
     display.drawString(0, 40, line2);
     display.display();
 }
 
 void setup() {
     delay(500); 
     Serial.begin(115200);
     unsigned long start = millis();
     while (!Serial && (millis() - start < 3000));
     
     LOG_INFO("SYS", "--- DONGLE BOOT START ---");
 
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF);
 
     Wire.begin(I2C_SDA, I2C_SCL);
 
     // ПИТАНИЕ (AXP2101)
     bool pmuFound = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
     
     if (!pmuFound) {
         LOG_ERROR("PMU", "AXP2101 not found! Check I2C hardware.");
     } else {
         LOG_INFO("PMU", "AXP2101 initialized successfully.");
         pmu.setALDO2Voltage(3300);
         pmu.enableALDO2();
 
         pmu.setALDO3Voltage(3300); 
         pmu.enableALDO3();
         LOG_INFO("PMU", "GPS Power (ALDO3) set to 3.3V and enabled.");
 
         pmu.disableALDO4();
         pmu.enableSystemVoltageMeasure();
         pmu.enableVbusVoltageMeasure();
         pmu.enableBattVoltageMeasure();
     }
 
     // ЭКРАН
     display.init();
     display.flipScreenVertically();
     display.setFont(ArialMT_Plain_16);
     showStatus("System Init...", "Power OK");
 
     // ИНИЦИАЛИЗАЦИЯ GPS
     GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
     LOG_INFO("GPS", "Serial port opened at 9600 baud (RX:%d, TX:%d)", GPS_RX, GPS_TX);
 
     for(int i=0; i<3; i++) {
         digitalWrite(LED_PIN, LED_ON);
         delay(100);
         digitalWrite(LED_PIN, LED_OFF);
         delay(100);
     }
     
     LOG_INFO("SYS", "--- SYSTEM READY ---");
 }
 
 void loop() {
     while (GPS_Serial.available() > 0) {
         gps.encode(GPS_Serial.read());
     }
 
     static uint32_t lastUpdate = 0;
     if (millis() - lastUpdate > 5000) { // Логируем ГПС раз в 5 секунд, чтобы не спамить
         lastUpdate = millis();
         
         digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        //  HDOP: Если число меньше 2.0 — точность отличная. Больше 5.0 — координаты могут «плавать».
        //  Altitude: Твоя высота над уровнем моря.
        //  Satellites: Реальное количество спутников, которые модуль видит прямо сейчас.         
         if (gps.location.isValid()) {
             LOG_INFO("GPS", "Fix OK! Sats: %d | HDOP: %.1f | Alt: %.1fm", 
                      gps.satellites.value(), 
                      gps.hdop.hdop(), 
                      gps.altitude.meters());
             LOG_INFO("GPS", "Pos: %.6f, %.6f | Time: %02d:%02d:%02d", 
                      gps.location.lat(), 
                      gps.location.lng(),
                      gps.time.hour(),
                      gps.time.minute(),
                      gps.time.second());
             
             showStatus("GPS FIX OK", "Sats: " + String(gps.satellites.value()));
         } else if (gps.satellites.value() > 0) {
             LOG_WARN("GPS", "Wait for Fix... Sats found: %d", gps.satellites.value());
             showStatus("Searching...", "Sats: " + String(gps.satellites.value()));
         } else {
             LOG_ERROR("GPS", "No satellites in view. Check antenna!");
             showStatus("GPS ERROR", "No Satellites");
         }
     }
 }