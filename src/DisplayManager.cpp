/**
 * File: DisplayManager.cpp
 * Version: 1.0.0
 * Description: Реализация класса управления дисплеем.
 */
 #include "DisplayManager.h"

 DisplayManager::DisplayManager(uint8_t address, int sda, int scl) 
     : _display(address, sda, scl) {
 }
 
 void DisplayManager::init() {
     _display.init();
     _display.flipScreenVertically();
 }
 
 void DisplayManager::showLogo() {
     _display.clear();
     _display.setFont(ArialMT_Plain_16);
     _display.drawString(0, 0,  "Naviga-Dongle");
     _display.drawString(0, 22, "System Init...");
     _display.drawString(0, 44, "Please Wait");
     _display.display();
     delay(2000);
 }
 
 void DisplayManager::showStatus(const String& line1, const String& line2, const String& line3, const String& line4) {
     _display.clear();
     _display.setFont(ArialMT_Plain_16);
     _display.drawString(0, 0,  line1);
     _display.drawString(0, 16, line2);
     _display.drawString(0, 32, line3);
     _display.drawString(0, 48, line4);
     _display.display();
 }