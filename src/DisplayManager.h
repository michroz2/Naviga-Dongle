/**
 * File: DisplayManager.h
 * Version: 1.46.3
 * Изменение: Исправление сигнатур методов для полного соответствия DisplayManager.cpp.
 * Удалено дублирующее определение BleStatus.
 * Description: Заголовочный файл менеджера дисплея.
 */

 #ifndef DISPLAY_MANAGER_H
 #define DISPLAY_MANAGER_H
 
 #include <Arduino.h>
 #include "BleProtocol.h" // Единый источник BleStatus
 
 // Размеры экрана
 #define SCREEN_WIDTH 128
 #define SCREEN_HEIGHT 64
 #define OLED_RESET    -1 
 
 class DisplayManager {
 public:
     // Сигнатура исправлена на (uint8_t, int, int) согласно ошибке в .cpp
     DisplayManager(uint8_t address, int sda, int scl);
     
     void init();
     void showLogo();
     
     /**
      * Обновление основного экрана рабочего режима.
      */
     void updateMainScreen(const char* macSuffix, bool gpsValid, int satellites, 
                          uint8_t myId, uint8_t msgSeq, uint8_t nodesCount,
                          bool hasTarget, uint8_t targetId, int distance, int azimuth, int quality,
                          BleStatus bleStatus);
 
     /**
      * Вывод статусной информации в 4 строки.
      * Исправлено: передача по константной ссылке согласно требованию реализации в .cpp
      */
     void showStatus(const String& line1, const String& line2, const String& line3, const String& line4);
     
     void toggleLed(); // Мигание системным светодиодом
 
 private:
     // Сохраняем указатель void* для совместимости с вашей скрытой реализацией
     void* _display; 
     uint8_t _address;
     bool _isLedOn;
 };
 
 #endif // DisplayManager.h