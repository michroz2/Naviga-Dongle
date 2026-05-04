/**
 * File: DisplayManager.cpp
 * Version: 1.32 Изменение: Перестановка строк на дисплее, вывод MAC-суффикса.
 */
 #include "DisplayManager.h"

  DisplayManager::DisplayManager(uint8_t address, int sda, int scl) 
 #if HAS_DISPLAY
     : _display(address, sda, scl) 
 #endif
 {}
 
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
     _display.drawString(0, 44, "BLE: NimBLE Ready");
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
 
 void DisplayManager::updateMainScreen(const char* macSuffix, bool gpsValid, int sats, uint8_t myNodeId, uint8_t myMsgSeq, 
    uint8_t activeNodes, bool hasTarget, uint8_t targetId, 
    int targetDist, int targetAzimuth, int targetQuality,
    BleStatus bleStatus) {
#if HAS_DISPLAY
String line1, line2, line3, line4;

// ИЗМЕНЕНИЕ 1.32: Строка 1 теперь Идентификация (MAC-ID-SEQ)
line1 = String(macSuffix) + "-" + String(myNodeId) + "-" + String(myMsgSeq);

// ИЗМЕНЕНИЕ 1.32: Строка 2 теперь GPS + Статус BLE
String bleLabel = "";
switch(bleStatus) {
case BLE_OFF:       bleLabel = " [-]"; break;
case BLE_UNPAIRED:  bleLabel = " [?]"; break; 
case BLE_CONNECTED: bleLabel = " [+]"; break;
}

if (!gpsValid) {
line2 = (sats > 0) ? ("GPS Wait " + String(sats)) : "GPS ERR";
} else {
line2 = "GPS OK " + String(sats);
}
line2 += bleLabel;

uint8_t neighbors = (activeNodes > 0) ? (activeNodes - 1) : 0;
line3 = "Neighbors: " + String(neighbors);

if (hasTarget) {
line4 = String(targetId) + ": " + String(targetDist) + "m/" + String(targetAzimuth) + "dg";
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