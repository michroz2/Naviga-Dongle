/**
 * File: GpsManager.h
 * Version: 1.19 Изменение: Поддержка статических координат для работы Ретранслятора без GPS (Шаг 2).
 * Description: Изолированный класс для управления GPS-модулем.
 */
 #ifndef GPS_MANAGER_H
 #define GPS_MANAGER_H
 
 #include <Arduino.h>
 #include <TinyGPS++.h>
 #include "configuration.h"
 
 // Определяем типы коллбэков для связи с внешним миром (UI и Питание)
 typedef void (*GpsStatusCallback)(String, String, String, String);
 typedef void (*GpsPowerCycleCallback)();
 
 class GpsManager {
 public:
     GpsManager();
 
     // Инициализация с передачей функций для вывода на экран и сброса питания
     void init(GpsStatusCallback statusCb, GpsPowerCycleCallback powerCb);
     
     // Обновление данных (должно вызываться в loop)
     void update();
 
     // Установка статических координат (для режима Ретранслятора)
     void setStaticLocation(float lat, float lon);
 
     // Простые геттеры для получения данных (БЕЗ const, так как TinyGPS++ методы не константные)
     bool isValid();
     float getLat();
     float getLon();
     uint32_t getSatellites();
     float getSpeed();           //Получение аппаратной скорости (Доплер)
 
     // Вспомогательные функции для математики (геометрия)
     float distanceTo(float lat, float lon);
     float courseTo(float lat, float lon);
     
 private:
     HardwareSerial gpsSerial; // Аппаратный UART для общения с модулем
     TinyGPSPlus tinyGps;      // Парсер NMEA
 
     float _staticLat = 0.0f;
     float _staticLon = 0.0f;
 
     // Внутренний метод проверки наличия потока NMEA
     bool checkNMEA(uint32_t baud);
 };
 
 #endif // GPS_MANAGER_H