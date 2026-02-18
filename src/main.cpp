/**
 * Project: Dongle (T-Beam v1.1 Custom E22)
 * File: main.cpp
 * Description: Display Navigation UI + Full GPS Logging
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <XPowersLib.h>       
 #include <TinyGPS++.h>
 #include "SSD1306Wire.h"
 #include "configuration.h"
 #include "logger.h"           
 
 // --- ОБЪЕКТЫ ---
 XPowersAXP2101 pmu;              
 SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);
 TinyGPSPlus gps;
 HardwareSerial GPS_Serial(1);
 
 // Переменные для навигации
 float dist = 0.0;
 float azmt = 0.0;
 
 // --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---
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
 
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF);
 
     Wire.begin(I2C_SDA, I2C_SCL);
 
     // ПИТАНИЕ (AXP2101)
     LOG_INFO("PMU", "Initializing AXP2101...");
     bool pmuFound = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
     
     if (!pmuFound) {
         LOG_ERROR("PMU", "AXP2101 not found!");
     } else {
         LOG_INFO("PMU", "AXP2101 initialized successfully.");
         pmu.setALDO2Voltage(3300);
         pmu.enableALDO2();
         pmu.setALDO3Voltage(3300); 
         pmu.enableALDO3();
         pmu.disableALDO4();
         pmu.enableSystemVoltageMeasure();
         pmu.enableVbusVoltageMeasure();
         pmu.enableBattVoltageMeasure();
     }
 
     // ЭКРАН
     display.init();
     display.flipScreenVertically();
     showStatus("System Init...", "Power OK", "Waiting GPS...");
 
     // ИНИЦИАЛИЗАЦИЯ GPS
     GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
     LOG_INFO("GPS", "Serial port opened at 9600 baud.");
 
     for(int i=0; i<3; i++) {
         digitalWrite(LED_PIN, LED_ON);
         delay(100);
         digitalWrite(LED_PIN, LED_OFF);
         delay(100);
     }
 }
 
 void loop() {
     while (GPS_Serial.available() > 0) {
         gps.encode(GPS_Serial.read());
     }
 
     static uint32_t lastUpdate = 0;
     if (millis() - lastUpdate > 1000) { 
         lastUpdate = millis();
         
         digitalWrite(LED_PIN, !digitalRead(LED_PIN));
         
         String gpsStatus;
         int sats = gps.satellites.value();
 
         // 1. Логика GPS
         if (gps.location.isValid()) {
             // Для ДИСПЛЕЯ: Короткая строка
             gpsStatus = "GPS OK " + String(sats);
             
             // изменено begin
             // Для ЛОГА: Полная информация (возвращаем как было)
             LOG_INFO("GPS", "Fix OK! Sats: %d | HDOP: %.1f | Alt: %.1fm", 
                      sats, 
                      gps.hdop.hdop(), 
                      gps.altitude.meters());
             LOG_INFO("GPS", "Pos: %.6f, %.6f | Time: %02d:%02d:%02d", 
                      gps.location.lat(), 
                      gps.location.lng(),
                      gps.time.hour(),
                      gps.time.minute(),
                      gps.time.second());
             // изменено end
 
         } else if (sats > 0) {
             gpsStatus = "GPS Wait " + String(sats);
             LOG_WARN("GPS", "Wait for Fix... Sats: %d", sats);
         } else {
             gpsStatus = "GPS ERROR";
             LOG_ERROR("GPS", "No satellites.");
         }
 
         // 2. Строки Дистанции и Азимута
         String distStr = "Dist: " + String(dist, 1);
         String azmtStr = "Azmt: " + String(azmt, 1);
 
         // 3. Вывод на дисплей
         showStatus(gpsStatus, distStr, azmtStr);
     }
 }