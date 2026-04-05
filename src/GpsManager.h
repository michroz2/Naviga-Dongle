/**
 * File: GpsManager.h
 * Version: 1.8 Изменение: Добавлен метод getSpeed() для адаптивной телеметрии.
 * Description: Изолированный класс для управления GPS-модулем.
 */
  #ifndef GPS_MANAGER_H
 #define GPS_MANAGER_H
 
 #include <Arduino.h>
 #include <TinyGPS++.h>
 #include "configuration.h"
 
 // Определяем типы коллбэков для связи с внешним миром
 typedef void (*GpsStatusCallback)(String, String, String, String);
 typedef void (*GpsPowerCycleCallback)();
 
 class GpsManager {
    public:
    GpsManager();

    // Инициализация с передачей функций для вывода на экран и сброса питания
    void init(GpsStatusCallback statusCb, GpsPowerCycleCallback powerCb);
    
    // Обновление данных (должно вызываться в loop)
    void update();

    // Простые геттеры для получения данных (БЕЗ const!)
    bool isValid();
    float getLat();
    float getLon();
    uint32_t getSatellites();
    float getSpeed();           //Получение аппаратной скорости (Доплер)

    // Вспомогательные функции для математики (БЕЗ const!)
    float distanceTo(float lat, float lon);
    float courseTo(float lat, float lon);
     
 private:
     HardwareSerial gpsSerial;
     TinyGPSPlus tinyGps;
 
     bool checkNMEA(uint32_t baud);
 };
 
 #endif // GPS_MANAGER_H