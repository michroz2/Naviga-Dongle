/**
 * File: GpsManager.cpp
 * Version: 1.46.7
 * Изменение: Авто-запись фикса в RAM-опору и разделение логики hasFix() / hasAnchor().
 * Description: Реализация класса управления GPS.
 */
 #include "GpsManager.h"
 #include "logger.h" 
 
 const uint32_t baudRates[] = {9600, 115200, 38400, 57600, 19200, 4800};
 const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);
 
 const uint8_t UBX_FACTORY_RESET[] = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A };
 
 GpsManager::GpsManager() : gpsSerial(1) {}
 
 bool GpsManager::checkNMEA(uint32_t baud) {
     gpsSerial.end(); 
     gpsSerial.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
     
     unsigned long start = millis();
     char prevChar = 0;
     while (millis() - start < 1500) { 
         if (gpsSerial.available()) {
             char c = gpsSerial.read();
             if (prevChar == '$' && (c == 'G' || c == 'P')) return true;
             prevChar = c;
         }
     }
     return false;
 }
 
 void GpsManager::init(GpsStatusCallback statusCb, GpsPowerCycleCallback powerCb) {
     if (statusCb) statusCb("Init GPS...", "Searching module", "Wait...", "");
     bool nmeaFound = false;
     uint32_t activeBaud = 0;
     
     for (int i = 0; i < numBauds; i++) {
         if (checkNMEA(baudRates[i])) {
             activeBaud = baudRates[i];
             nmeaFound = true;
             break;
         }
     }
     
     if (nmeaFound) {
         if (activeBaud == 9600) return; 
         else {
             if (statusCb) statusCb("Init GPS...", "Switching Baud", String(activeBaud) + " -> 9600", "");
             gpsSerial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n");
             gpsSerial.flush();
             delay(500); 
             if (checkNMEA(9600)) return; 
             else nmeaFound = false; 
         }
     }
     
     if (!nmeaFound) {
         if (statusCb) statusCb("Init GPS...", "Rescue Mode!", "Wait 10 sec...", "");
         for (int i = 0; i < numBauds; i++) {
             gpsSerial.end(); 
             gpsSerial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
             delay(50);
             for(int j = 0; j < 3; j++) { gpsSerial.write(UBX_FACTORY_RESET, sizeof(UBX_FACTORY_RESET)); gpsSerial.flush(); delay(50); }
         }
         
         if (powerCb) powerCb(); 
         
         if (checkNMEA(115200)) { gpsSerial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n"); gpsSerial.flush(); delay(500); } 
         else if (checkNMEA(9600)) { } 
         
         gpsSerial.end(); 
         gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); 
     }
 }
 
 void GpsManager::update() {
     while (gpsSerial.available() > 0) {
         tinyGps.encode(gpsSerial.read()); 
     }
     
     // ИЗМЕНЕНИЕ 1.46.7: Если есть свежий спутниковый фикс, автоматически
     // перезаписываем опорную RAM-точку актуальными координатами.
     if (tinyGps.location.isValid() && tinyGps.location.isUpdated()) {
         _anchorLat = tinyGps.location.lat();
         _anchorLon = tinyGps.location.lng();
     }
 }
 
 void GpsManager::setAnchorLocation(float lat, float lon) {
     _anchorLat = lat;
     _anchorLon = lon;
     if (_anchorLat != 0.0f || _anchorLon != 0.0f) {
         LOG_INFO("GPS", "RAM Anchor set: Lat=%.6f, Lon=%.6f", _anchorLat, _anchorLon);
     }
 }
 
 bool GpsManager::hasFix() {
     return tinyGps.location.isValid();
 }
 
 bool GpsManager::hasAnchor() {
     if (tinyGps.location.isValid()) return true;
     if (_anchorLat != 0.0f || _anchorLon != 0.0f) return true;
     return false;
 }
 
 bool GpsManager::isValid() { 
     return hasAnchor(); 
 }
 
 float GpsManager::getLat() { 
     if (tinyGps.location.isValid()) return tinyGps.location.lat();
     return _anchorLat;
 }
 
 float GpsManager::getLon() { 
     if (tinyGps.location.isValid()) return tinyGps.location.lng();
     return _anchorLon;
 }
 
 uint32_t GpsManager::getSatellites() { return tinyGps.satellites.value(); }
 
 float GpsManager::getSpeed() {
     if (!tinyGps.location.isValid() && (_anchorLat != 0.0f || _anchorLon != 0.0f)) return 0.0f;
     return tinyGps.speed.kmph();
 } 
 
 float GpsManager::distanceTo(float lat, float lon) {
     return TinyGPSPlus::distanceBetween(getLat(), getLon(), lat, lon);
 }
 
 float GpsManager::courseTo(float lat, float lon) {
     return TinyGPSPlus::courseTo(getLat(), getLon(), lat, lon);
 }