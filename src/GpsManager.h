/**
 * File: GpsManager.h
 * Version: 1.46.7
 * Изменение: Разделение понятий спутникового фикса (hasFix) и наличия опорной RAM-точки (hasAnchor).
 * Description: Изолированный класс для управления GPS-модулем.
 */
 #ifndef GPS_MANAGER_H
 #define GPS_MANAGER_H
 
 #include <Arduino.h>
 #include <TinyGPS++.h>
 #include "configuration.h"
 
 typedef void (*GpsStatusCallback)(String, String, String, String);
 typedef void (*GpsPowerCycleCallback)();
 
 class GpsManager {
 public:
     GpsManager();
 
     void init(GpsStatusCallback statusCb, GpsPowerCycleCallback powerCb);
     void update();
 
     // НОВОЕ 1.46.7: Явная установка опорной точки (через BLE или дефолты)
     void setAnchorLocation(float lat, float lon);
 
     // НОВОЕ 1.46.7: Разделенные геттеры состояний
     bool hasFix();    // Есть ли честный спутниковый фикс прямо сейчас
     bool hasAnchor(); // Доступна ли хоть какая-то привязка для GeoPacker
 
     bool isValid();   // Оставлен для обратной совместимости, эквивалентен hasAnchor
     float getLat();   // Выдает GPS Lat, если есть фикс, иначе _anchorLat
     float getLon();   // Выдает GPS Lon, если есть фикс, иначе _anchorLon
     
     uint32_t getSatellites();
     float getSpeed(); 
 
     float distanceTo(float lat, float lon);
     float courseTo(float lat, float lon);
     
 private:
     HardwareSerial gpsSerial; 
     TinyGPSPlus tinyGps;      
 
     float _anchorLat = 0.0f;
     float _anchorLon = 0.0f;
 
     bool checkNMEA(uint32_t baud);
 };
 
 #endif // GPS_MANAGER_H