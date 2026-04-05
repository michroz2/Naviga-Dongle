/**
 * File: GpsManager.cpp
 * Version: 1.8 Изменение: Реализация метода getSpeed().
 * Description: Реализация класса управления GPS.
 */
 #include "GpsManager.h"

 const uint32_t baudRates[] = {9600, 115200, 38400, 57600, 19200, 4800};
 const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);
 const uint8_t UBX_FACTORY_RESET[] = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A };
 
 // Инициализируем Serial-порт номер 1
 GpsManager::GpsManager() : gpsSerial(1) {}
 
 bool GpsManager::checkNMEA(uint32_t baud) {
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
             gpsSerial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
             delay(50);
             for(int j = 0; j < 3; j++) { gpsSerial.write(UBX_FACTORY_RESET, sizeof(UBX_FACTORY_RESET)); gpsSerial.flush(); delay(50); }
         }
         
         // Вызываем внешний сброс питания (из main.cpp)
         if (powerCb) powerCb(); 
         
         if (checkNMEA(115200)) { gpsSerial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n"); gpsSerial.flush(); delay(500); } 
         else if (checkNMEA(9600)) { } 
         gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
     }
 }
 
 void GpsManager::update() {
     while (gpsSerial.available() > 0) {
         tinyGps.encode(gpsSerial.read());
     }
 }
 
bool GpsManager::isValid() { return tinyGps.location.isValid(); }
float GpsManager::getLat() { return tinyGps.location.lat(); }
float GpsManager::getLon() { return tinyGps.location.lng(); }
uint32_t GpsManager::getSatellites() { return tinyGps.satellites.value(); }

//Метод получения скорости
float GpsManager::getSpeed() {
    return tinyGps.speed.kmph();
} // GpsManager::getSpeed()

float GpsManager::distanceTo(float lat, float lon) {
    return TinyGPSPlus::distanceBetween(getLat(), getLon(), lat, lon);
}

float GpsManager::courseTo(float lat, float lon) {
    return TinyGPSPlus::courseTo(getLat(), getLon(), lat, lon);
}