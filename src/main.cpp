/**
 * Project: Dongle (T-Beam v1.1 Custom E22)
 * File: main.cpp
 * Description: Hardware Initialization (Power & Display)
 * Stack: Meshtastic Compatible (XPowersLib, SSD1306)
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <XPowersLib.h>
 #include "SSD1306Wire.h"      // OLED Driver
 #include "configuration.h"    // Our Custom Pinout
 
 // --- ОБЪЕКТЫ ---
 XPowersAXP2101 pmu;              // Используем конкретный класс AXP2101
 SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL); // Экран
 
 // --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---
 
 // Функция вывода текста на экран (чтобы не дублировать код)
 void showStatus(String line1, String line2) {
     display.clear();
     display.drawString(0, 0, "DONGLE v1.0");
     display.drawString(0, 20, line1);
     display.drawString(0, 40, line2);
     display.display();
 }
 
 void setup() {
     // 1. Запуск Serial для отладки
     // Добавляем задержку перед инициализацией Serial для стабильности ESP32 при ребуте
     delay(500); 
     Serial.begin(115200);
     // Ждем, но проверяем таймер, чтобы не зависнуть
     unsigned long start = millis();
     while (!Serial && (millis() - start < 3000));
     
     Serial.println("\n--- DONGLE BOOT START ---");
 
     // НАСТРОЙКА LED (RED, GPIO 4)
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF); // Сразу гасим (HIGH)
 
     // 2. Инициализация I2C шины
     Wire.begin(I2C_SDA, I2C_SCL);
 
     // 3. ИНИЦИАЛИЗАЦИЯ ПИТАНИЯ (AXP2101)
     // Это критический этап. Без этого GPS и LoRa мертвы.
     Serial.print("PMU: Initializing AXP2101...");
     
     bool pmuFound = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
     
     if (!pmuFound) {
         Serial.println("FAILED!");
         Serial.println("CRITICAL ERROR: PMU not found. Check if board is AXP192 or AXP2101.");
     } else {
         Serial.println("OK");
 
         // Настройка напряжений (как в Meshtastic для T-Beam 1.1)
         
         // ALDO2 -> Питает LoRa и Экран (обычно)
         pmu.setALDO2Voltage(3300);
         pmu.enableALDO2();
         Serial.println("PMU: LoRa/Screen Power (ALDO2) -> ON");
 
         // ALDO3 -> Питает GPS
         pmu.setALDO3Voltage(3300);
         pmu.enableALDO3();
         Serial.println("PMU: GPS Power (ALDO3) -> ON");
 
         // ALDO4 -> Обычно не используется или для датчиков
         pmu.disableALDO4();
 
         // Включаем мониторинг батареи
         pmu.enableSystemVoltageMeasure();
         pmu.enableVbusVoltageMeasure();
         pmu.enableBattVoltageMeasure();
     }
 
     // 4. ИНИЦИАЛИЗАЦИЯ ЭКРАНА
     Serial.print("Display: Initializing...");
     if(!display.init()) {
         Serial.println("FAILED!");
     } else {
         Serial.println("OK");
         display.flipScreenVertically();
         display.setFont(ArialMT_Plain_16);
         showStatus("System Init...", "Power OK");
     }
 
     // 5. Тест LED при старте (3 быстрых мигания)
     for(int i=0; i<3; i++) {
         digitalWrite(LED_PIN, LED_ON);
         delay(100);
         digitalWrite(LED_PIN, LED_OFF);
         delay(100);
     }
     
     Serial.println("--- SYSTEM READY ---");
     delay(1000);
 }
 
 void loop() {
     // Простейшая проверка жизни (Heartbeat)
     static uint32_t lastBlink = 0;
     
     if (millis() - lastBlink > 1000) {
         lastBlink = millis();
         
         // Моргаем синим светодиодом
         bool ledState = digitalRead(LED_PIN);
         digitalWrite(LED_PIN, !ledState);
         
         // Выводим статус батареи в Serial
         if (pmu.isBatteryConnect()) {
             Serial.printf("Battery: %d%% (%.2fV)\n", pmu.getBatteryPercent(), pmu.getBattVoltage() / 1000.0);
         } else {
             Serial.println("Battery: None (USB Power)");
         }
         
         // Обновляем экран
         String powerSrc = pmu.isVbusIn() ? "USB Power" : "Battery";
         int batPct = pmu.isBatteryConnect() ? pmu.getBatteryPercent() : 0;
         showStatus(powerSrc, String(batPct) + "%");
     }
 }