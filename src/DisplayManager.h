/**
 * File: DisplayManager.h
 * Version: 1.20 Изменение: Добавлена условная компиляция и методы инкапсуляции LED/сборки строк.
 * Description: Изолированный класс для управления OLED дисплеем (SSD1306) и системным LED.
 */
 #ifndef DISPLAY_MANAGER_H
 #define DISPLAY_MANAGER_H
 
 #include <Arduino.h>
 #include "configuration.h"
 
 // Библиотека дисплея подключается только если дисплей активирован
 #if HAS_DISPLAY
 #include "SSD1306Wire.h"
 #endif
 
 class DisplayManager {
 public:
     // Конструктор принимает адрес и пины I2C
     DisplayManager(uint8_t address, int sda, int scl);
 
     // Базовая настройка (инициализирует также LED, если включен)
     void init();
     
     // Показ стартового логотипа
     void showLogo();
     
     // Вывод четырех строк состояния (кастомный вывод)
     void showStatus(const String& line1, const String& line2, const String& line3, const String& line4);
 
     // НОВОЕ 1.20: Метод для инкапсулированной сборки и вывода главного экрана
     void updateMainScreen(bool gpsValid, int sats, uint8_t myNodeId, uint8_t myMsgSeq, 
                           uint8_t activeNodes, bool hasTarget, uint8_t targetId, 
                           int targetDist, int targetAzimuth, int targetQuality);
 
     // НОВОЕ 1.20: Инкапсулированное управление статусным светодиодом
     void toggleLed();
 
 private:
 #if HAS_DISPLAY
     SSD1306Wire _display; // Внутренний объект библиотеки
 #endif
 };
 
 #endif // DISPLAY_MANAGER_H