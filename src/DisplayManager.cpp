/**
 * File: DisplayManager.cpp
 * Version: 1.20 Изменение: Реализация методов сборки интерфейса и условной компиляции.
 * Description: Реализация класса управления дисплеем и LED.
 */
 #include "DisplayManager.h"

 DisplayManager::DisplayManager(uint8_t address, int sda, int scl) 
 #if HAS_DISPLAY
     : _display(address, sda, scl) 
 #endif
 {
     // Пустое тело, инициализация списка членов делает всю работу
 }
 
 void DisplayManager::init() {
 #if HAS_STATUS_LED
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF);
 #endif
 
 #if HAS_DISPLAY
     _display.init();
     _display.flipScreenVertically();
 #endif
 }
 
 void DisplayManager::showLogo() {
 #if HAS_DISPLAY
     _display.clear();
     _display.setFont(ArialMT_Plain_16);
     _display.drawString(0, 0,  "Naviga-Dongle");
     _display.drawString(0, 22, "System Init...");
     _display.drawString(0, 44, "Please Wait");
     _display.display();
     delay(2000);
 #endif
 }
 
 void DisplayManager::showStatus(const String& line1, const String& line2, const String& line3, const String& line4) {
 #if HAS_DISPLAY
     _display.clear();
     _display.setFont(ArialMT_Plain_16);
     _display.drawString(0, 0,  line1);
     _display.drawString(0, 16, line2);
     _display.drawString(0, 32, line3);
     _display.drawString(0, 48, line4);
     _display.display();
 #endif
 }
 
 void DisplayManager::updateMainScreen(bool gpsValid, int sats, uint8_t myNodeId, uint8_t myMsgSeq, 
                                       uint8_t activeNodes, bool hasTarget, uint8_t targetId, 
                                       int targetDist, int targetAzimuth, int targetQuality) {
 #if HAS_DISPLAY
     String line1, line2, line3, line4;
 
     // Сборка Строки 1 (Статус GPS)
     if (!gpsValid) {
         line1 = (sats > 0) ? ("GPS Wait " + String(sats)) : "GPS ERROR";
     } else {
         line1 = "GPS OK " + String(sats);
     }
 
     // Сборка Строки 2 (Собственный статус)
     line2 = "My: " + String(myNodeId) + "-" + String(myMsgSeq);
     
     // Сборка Строки 3 (Соседи)
     uint8_t neighbors = (activeNodes > 0) ? (activeNodes - 1) : 0;
     line3 = "Neighbors: " + String(neighbors);
     
     // Сборка Строки 4 (Цель)
     if (hasTarget) {
         line4 = String(targetId) + ": " + 
                 String(targetDist) + "m/" + 
                 String(targetAzimuth) + "/" + 
                 String(targetQuality);
     } else {
         line4 = "No targets";
     } 
 
     showStatus(line1, line2, line3, line4);
 #endif
 }
 
 void DisplayManager::toggleLed() {
 #if HAS_STATUS_LED
     digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
 #endif
 }