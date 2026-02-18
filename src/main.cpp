/**
 * Project: Dongle (T-Beam v1.1 Custom E22)
 * File: main.cpp
 * Description: Display Navigation UI + Full GPS Logging + Full LoRa Init
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
 
 // --- ОБЪЕКТЫ ---
 XPowersAXP2101 pmu;              
 SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);
 TinyGPSPlus gps;
 HardwareSerial GPS_Serial(1);
 
 // Радиомодуль E22 (SX1268)
 SX1268 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
 
 // Переменные навигации
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
 
     // 0. ОТКЛЮЧЕНИЕ ВСТРОЕННОГО МОДУЛЯ (SX1276)
     // Удержание CS в высоком уровне (HIGH) предотвращает конфликты на шине SPI
     pinMode(LORA_ONBOARD_CS, OUTPUT);
     digitalWrite(LORA_ONBOARD_CS, HIGH);
     LOG_INFO("SYS", "Onboard LoRa (CS:18) DISABLED.");
 
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF);
 
     Wire.begin(I2C_SDA, I2C_SCL);
 
     // 1. ИНИЦИАЛИЗАЦИЯ ШИНЫ SPI
     // Используются пины согласно кастомной разводке E22
     SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
     LOG_INFO("SYS", "SPI initialized (SCK:%d, MISO:%d, MOSI:%d, CS:%d)", LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
 
     // ПИТАНИЕ (AXP2101)
     LOG_INFO("PMU", "Initializing AXP2101...");
     bool pmuFound = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
     
     if (!pmuFound) {
         LOG_ERROR("PMU", "AXP2101 not found!");
     } else {
         LOG_INFO("PMU", "AXP2101 initialized.");
         pmu.setALDO2Voltage(3300); // Питание LoRa
         pmu.enableALDO2();
         pmu.setALDO3Voltage(3300); // Питание GPS
         pmu.enableALDO3();
         pmu.disableALDO4();
         pmu.enableSystemVoltageMeasure();
     }
 
     // ЭКРАН
     display.init();
     display.flipScreenVertically();
     showStatus("System Init...", "Power OK", "Init LoRa...");
 
     // GPS
     GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
     LOG_INFO("GPS", "Serial opened.");
 
     // 5. ИНИЦИАЛИЗАЦИЯ И ПОЛНАЯ НАСТРОЙКА LORA (E22 / SX1268)
     LOG_INFO("LORA", "Initializing SX1268...");
     
     // Базовый старт
     int state = radio.begin(); 
 
     if (state == RADIOLIB_ERR_NONE) {
         LOG_INFO("LORA", "Chip found!");
 
         // 5.1 Настройка частоты (433.0 MHz)
         radio.setFrequency(433.0);

         // 5.2 Настройка регулятора напряжения (DC-DC для высокой эффективности)
        //  state = radio.setRegulatorMode(RADIOLIB_SX126X_REGULATOR_DC_DC);
        //  if (state != RADIOLIB_ERR_NONE) {
        //      LOG_ERROR("LORA", "Regulator setup failed, code: %d", state);
        //  }

         // 5.3 Отключение управления RF-переключателем через пин DIO2
         // Управление осуществляется внешними пинами LORA_RXEN/LORA_TXEN
         radio.setDio2AsRfSwitch(false);


         // 5.4 Настройка модуляции (Explicit Configuration)
         // Bandwidth: 125 kHz, SF: 9, CR: 4/5
         radio.setBandwidth(125.0);
         radio.setSpreadingFactor(9);
         radio.setCodingRate(5);
         radio.setSyncWord(0x12); // Private Network
 
         // 5.5 Настройка выходной мощности
         // +22 dBm - это максимум для чипа SX1268. 
         // Внешний усилитель модуля E22 усилит это до +30..33 dBm.
         radio.setOutputPower(22);
         radio.setCurrentLimit(140); // Защита по току для самого чипа
 
         // 5.6 Настройка TCXO (1.8V) - Обязательно для стабильной работы E22
         if (radio.setTCXO(1.8) == RADIOLIB_ERR_INVALID_TCXO_VOLTAGE) {
              LOG_ERROR("LORA", "TCXO setup failed!");
         } else {
              LOG_INFO("LORA", "TCXO set to 1.8V");
         }
 
         // 5.7 Антенный переключатель (RXEN/TXEN)
         radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         
         // 5.8 Перевод в Standby (Готовность)
         radio.standby();
         LOG_INFO("LORA", "Setup Complete -> STANDBY");
         LOG_INFO("LORA", "Params: 433MHz, SF9, BW125, Pwr:22dBm(Max)");
 
     } else {
         LOG_ERROR("LORA", "Init FAILED, code: %d", state);
         showStatus("LoRa Error", "Code: " + String(state), "Check HW");
     }
 
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
 
         if (gps.location.isValid()) {
             gpsStatus = "GPS OK " + String(sats);
             LOG_INFO("GPS", "Fix OK! Sats: %d | HDOP: %.1f | Alt: %.1fm", 
                      sats, gps.hdop.hdop(), gps.altitude.meters());
             LOG_INFO("GPS", "Pos: %.6f, %.6f | Time: %02d:%02d:%02d", 
                      gps.location.lat(), gps.location.lng(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
         } else if (sats > 0) {
             gpsStatus = "GPS Wait " + String(sats);
             LOG_WARN("GPS", "Wait... Sats: %d", sats);
         } else {
             gpsStatus = "GPS ERROR";
             LOG_ERROR("GPS", "No satellites.");
         }
 
         showStatus(gpsStatus, "Dist: " + String(dist, 1), "Azmt: " + String(azmt, 1));
     }
 }