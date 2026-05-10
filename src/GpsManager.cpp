/**
 * File: GpsManager.cpp
 * Version: 1.36 Изменение: Добавлен вызов gpsSerial.end() перед begin() для предотвращения утечек ресурсов UART.
 * Description: Реализация класса управления GPS.
 */
 #include "GpsManager.h"
 #include "logger.h" // Подключено для логирования установки статических координат
 
 // Массив стандартных скоростей (Baud rates) для поиска GPS-модуля при автоопределении
 const uint32_t baudRates[] = {9600, 115200, 38400, 57600, 19200, 4800};
 const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);
 
 // Команда сброса к заводским настройкам для чипов Ublox (Rescue Mode)
 const uint8_t UBX_FACTORY_RESET[] = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A };
 
 // Инициализируем Serial-порт номер 1 для аппаратного UART
 GpsManager::GpsManager() : gpsSerial(1) {}
 
 // Функция проверки наличия валидного NMEA потока от GPS на заданной скорости
 bool GpsManager::checkNMEA(uint32_t baud) {
     // ИЗМЕНЕНИЕ 1.36: Гарантированное закрытие предыдущего соединения перед открытием нового,
     // чтобы избежать утечек памяти и зависаний аппаратного блока UART в ESP32
     gpsSerial.end(); 
     gpsSerial.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
     
     unsigned long start = millis();
     char prevChar = 0;
     while (millis() - start < 1500) { // Ждем 1.5 секунды
         if (gpsSerial.available()) {
             char c = gpsSerial.read();
             // Ищем начало NMEA предложения (например, $GPRMC, $GPGGA)
             if (prevChar == '$' && (c == 'G' || c == 'P')) return true;
             prevChar = c;
         }
     }
     return false;
 }
 
 // Инициализация менеджера GPS с автоопределением Baud rate и механизмом восстановления
 void GpsManager::init(GpsStatusCallback statusCb, GpsPowerCycleCallback powerCb) {
     if (statusCb) statusCb("Init GPS...", "Searching module", "Wait...", "");
     bool nmeaFound = false;
     uint32_t activeBaud = 0;
     
     // Пытаемся найти GPS-модуль на одной из известных скоростей
     for (int i = 0; i < numBauds; i++) {
         if (checkNMEA(baudRates[i])) {
             activeBaud = baudRates[i];
             nmeaFound = true;
             break;
         }
     }
     
     if (nmeaFound) {
         if (activeBaud == 9600) return; // Идеальная скорость, выходим
         else {
             // Если скорость отлична от 9600, пытаемся переконфигурировать Ublox модуль
             if (statusCb) statusCb("Init GPS...", "Switching Baud", String(activeBaud) + " -> 9600", "");
             gpsSerial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n");
             gpsSerial.flush();
             delay(500); 
             if (checkNMEA(9600)) return; 
             else nmeaFound = false; // Сброс не удался, идем в Rescue Mode
         }
     }
     
     // Если NMEA не найден или переключение скорости провалилось (Rescue Mode)
     if (!nmeaFound) {
         if (statusCb) statusCb("Init GPS...", "Rescue Mode!", "Wait 10 sec...", "");
         // Пытаемся отправить команду жесткого сброса (Factory Reset) на всех скоростях
         for (int i = 0; i < numBauds; i++) {
             // ИЗМЕНЕНИЕ 1.36: Очищаем ресурсы UART перед каждой сменой скорости в цикле Rescue Mode
             gpsSerial.end(); 
             gpsSerial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
             delay(50);
             for(int j = 0; j < 3; j++) { gpsSerial.write(UBX_FACTORY_RESET, sizeof(UBX_FACTORY_RESET)); gpsSerial.flush(); delay(50); }
         }
         
         // Вызываем внешний сброс питания (из main.cpp), если поддерживается железом (PMU)
         if (powerCb) powerCb(); 
         
         // Проверяем, ожил ли модуль после сброса питания
         if (checkNMEA(115200)) { gpsSerial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n"); gpsSerial.flush(); delay(500); } 
         else if (checkNMEA(9600)) { } 
         
         // ИЗМЕНЕНИЕ 1.36: Финальное закрытие перед установкой постоянной рабочей скорости
         gpsSerial.end(); 
         gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // Финально устанавливаем 9600
     }
 }
 
 // Постоянно вызывается в основном цикле для парсинга новых байт из UART
 void GpsManager::update() {
     while (gpsSerial.available() > 0) {
         tinyGps.encode(gpsSerial.read()); // Отдаем данные библиотеке TinyGPS++
     }
 }
 
 // Устанавливает статические координаты (например, для роли NODE_RELAY)
 void GpsManager::setStaticLocation(float lat, float lon) {
     _staticLat = lat;
     _staticLon = lon;
     if (_staticLat != 0.0f || _staticLon != 0.0f) {
         LOG_INFO("GPS", "Static location set: Lat=%.6f, Lon=%.6f", _staticLat, _staticLon);
     }
 }
 
 // Проверка валидности локации (учитывает статические координаты)
 bool GpsManager::isValid() { 
     if (tinyGps.location.isValid()) return true;
     if (_staticLat != 0.0f || _staticLon != 0.0f) return true;
     return false; 
 }
 
 // Получение широты (физической или статической)
 float GpsManager::getLat() { 
     if (tinyGps.location.isValid()) return tinyGps.location.lat();
     return _staticLat;
 }
 
 // Получение долготы (физической или статической)
 float GpsManager::getLon() { 
     if (tinyGps.location.isValid()) return tinyGps.location.lng();
     return _staticLon;
 }
 
 uint32_t GpsManager::getSatellites() { return tinyGps.satellites.value(); }
 
 // Метод получения скорости (основан на аппаратном Доплеровском эффекте чипа)
 float GpsManager::getSpeed() {
     // Если нет фикса, но заданы статические координаты, скорость всегда 0 (стационарный узел)
     if (!tinyGps.location.isValid() && (_staticLat != 0.0f || _staticLon != 0.0f)) return 0.0f;
     return tinyGps.speed.kmph();
 } // GpsManager::getSpeed()
 
 // Расчет дистанции в метрах до другой точки на основе сферической геометрии (Haversine)
 float GpsManager::distanceTo(float lat, float lon) {
     return TinyGPSPlus::distanceBetween(getLat(), getLon(), lat, lon);
 }
 
 // Расчет азимута в градусах до другой точки
 float GpsManager::courseTo(float lat, float lon) {
     return TinyGPSPlus::courseTo(getLat(), getLon(), lat, lon);
 } //GPSMANAGER.CPP