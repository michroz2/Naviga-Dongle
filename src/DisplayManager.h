/**
 * File: DisplayManager.h
 * Version: 1.26 Изменение: Добавлена поддержка отображения статуса BLE.
 */
 #ifndef DISPLAY_MANAGER_H
 #define DISPLAY_MANAGER_H
 
 #include <Arduino.h>
 #include "configuration.h"
 
 #if HAS_DISPLAY
 #include "SSD1306Wire.h"
 #endif
 
 // Состояния BLE для отображения
 enum BleStatus {
     BLE_OFF,
     BLE_UNPAIRED,
     BLE_CONNECTED
 };
 
 class DisplayManager {
 public:
     DisplayManager(uint8_t address, int sda, int scl);
     void init();
     void showLogo();
     void showStatus(const String& line1, const String& line2, const String& line3, const String& line4);
 
     // Добавлен параметр bleStatus
     void updateMainScreen(bool gpsValid, int sats, uint8_t myNodeId, uint8_t myMsgSeq, 
                           uint8_t activeNodes, bool hasTarget, uint8_t targetId, 
                           int targetDist, int targetAzimuth, int targetQuality,
                           BleStatus bleStatus);
 
     void toggleLed();
 
 private:
 #if HAS_DISPLAY
     SSD1306Wire _display;
 #endif
 };
 
 #endif