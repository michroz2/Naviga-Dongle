/**
 * File: DisplayManager.h
 * Version: 1.0.0
 * Description: Изолированный класс для управления OLED дисплеем (SSD1306).
 */
 #ifndef DISPLAY_MANAGER_H
 #define DISPLAY_MANAGER_H
 
 #include <Arduino.h>
 #include "SSD1306Wire.h"
 
 class DisplayManager {
 public:
     // Конструктор принимает адрес и пины I2C
     DisplayManager(uint8_t address, int sda, int scl);
 
     // Базовая настройка
     void init();
     
     // Показ стартового логотипа
     void showLogo();
     
     // Вывод четырех строк состояния
     void showStatus(const String& line1, const String& line2, const String& line3, const String& line4);
 
 private:
     SSD1306Wire _display; // Внутренний объект библиотеки
 };
 
 #endif // DISPLAY_MANAGER_H