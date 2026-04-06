# Project: Naviga_Dongle Snapshot V1.12

## File: .\src\configuration.h
```cpp
/** Test
 * File: configuration.h
 * Version: 1.12 Изменение: Добавлен таймаут кэширования топологии и порог плотности группы.
 * Description: Конфигурация пинов и базовых настроек.
 */
 
 #ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

// --- LED (RED) ---
#define LED_PIN     4   // Пин управления красным светодиодом на плате T-Beam v1.1
#define LED_ON      LOW // Уровень сигнала для включения светодиода (LOW = включен)
#define LED_OFF     HIGH // Уровень сигнала для выключения светодиода (HIGH = выключен)

// --- I2C PINS (OLED & AXP2101) ---
// Пины аппаратной шины I2C для связи с дисплеем и контроллером питания
#define I2C_SDA 21      // Линия данных (Serial Data)
#define I2C_SCL 22      // Линия тактирования (Serial Clock)

// --- POWER MANAGEMENT (AXP2101) ---
// Адрес I2C определяется программно, здесь задан пин аппаратного прерывания
#define PMU_IRQ 35      // Пин прерывания от контроллера питания (PMU).
                        // Примечание: На старых ревизиях может совпадать с DIO1.
                        // В данной конфигурации конфликт устранен (DIO1 вынесен).

// --- LORA (ONBOARD SX1276) ---
// Пин Chip Select (CS) для встроенного на плату модуля LoRa.
// Установка в HIGH необходима для его надежной изоляции от шины SPI.
#define LORA_ONBOARD_CS 18

// --- EBYTE E22-400M33S (SX1268) PINOUT ---
// Очистка возможных предыдущих определений макросов SPI перед их явным заданием
#undef LORA_CS
#undef LORA_RST
#undef LORA_BUSY
#undef LORA_DIO1
#undef LORA_SCK
#undef LORA_MISO
#undef LORA_MOSI

// Кастомная конфигурация пинов для управления внешним модулем EBYTE E22
#define LORA_CS     13  // Пин выбора чипа (Chip Select) для внешнего модуля E22
#define LORA_RST    14  // Пин аппаратного сброса (Reset) модуля E22
#define LORA_BUSY   15  // Пин состояния (Busy Line), критически важен для чипов серии SX126x
#define LORA_DIO1   35  // Пин прерываний (DIO1). Важно: GPIO35 аппаратно работает только на вход (Input Only)
#define LORA_TXEN   2   // Пин включения передатчика (TX Enable). Дублируется на синий светодиод платы
#define LORA_RXEN   25  // Пин включения приемника (RX Enable)

// --- SPI BUS ---
// Стандартные пины аппаратной шины SPI для T-Beam v1.1 (используются внешним модулем E22)
#define LORA_SCK    5   // Линия тактирования (Serial Clock)
#define LORA_MISO   19  // Линия приема данных (Master In Slave Out)
#define LORA_MOSI   27  // Линия передачи данных (Master Out Slave In)

// --- GPS (Ublox NEO-8M) ---
// Пины аппаратного UART интерфейса для связи с модулем GPS
#define GPS_RX      34  // Линия приема данных от GPS-модуля
#define GPS_TX      12  // Линия передачи команд в GPS-модуль

// --- ТАЙМЕРЫ И ИНТЕРВАЛЫ ---
const uint32_t txInterval = 10000;
const uint32_t gpsUpdateInterval = 1000;

#define CLEANUP_INTERVAL_MS 10000       // Интервал фоновой очистки Node db)
#define HEARTBEAT_INTERVAL_MS 2700000   // Интервал фоновой отправки NodeInfo: 45 минут (45 * 60 * 1000 мс)
#define MIN_GREETING_NODEINFO_JITTER 120000
#define MAX_GREETING_NODEINFO_JITTER 300000

// --- НАСТРОЙКИ АДАПТИВНОЙ ОТПРАВКИ КООРДИНАТ ---
#define MIN_MOVEMENT_METERS    15.0f  // Порог дистанции для фиксации движения (защита от дрейфа)
#define MIN_SPEED_KMPH         2.0f   // Минимальная скорость для подтверждения движения (защита от дрейфа)
#define SNEAK_MOVEMENT_METERS  40.0f  // Дистанция безусловной отправки (для медленного движения/крадущегося)
#define TX_INTERVAL_MOVING     5000   // Максимальная частота отправки в движении (5 сек)
#define TX_INTERVAL_STILL      300000 // Редкий пинг на стоянке (5 минут)

// --- НАСТРОЙКИ СЕТИ ---
#define DEFAULT_TTL 3 // Максимальное количество ретрансляций (прыжков) для одного пакета
#define MAX_DIRECT_CONNECT_METERS 200.0f 
#define TOPOLOGY_UPDATE_INTERVAL_MS 15000 // Интервал пересчета топологии (15 сек)

#endif // CONFIGURATION_H
```

---

## File: .\src\DisplayManager.cpp
```cpp
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
```

---

## File: .\src\DisplayManager.h
```cpp
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
```

---

## File: .\src\GeoPacker.cpp
```cpp
#include "GeoPacker.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GeoPacker::GeoPacker() : lonExtraScale(2.0f) {
} // Конец конструктора

void GeoPacker::updateLonScale(float currentLat) {
    float radians = currentLat * (M_PI / 180.0f);
    float cosLat = cosf(radians);

    // Ограничение для исключения деления на ноль вблизи полюсов
    if (cosLat < 0.01f) {
        cosLat = 0.01f;
    } // Конец проверки cosLat

    float rawScale = 1.0f / cosLat;
    
    // Округление множителя до сотых долей для стабильности между узлами
    lonExtraScale = roundf(rawScale * 100.0f) / 100.0f;
} // Конец метода updateLonScale

uint32_t GeoPacker::pack(float lat, float lon) const {
    int32_t latI = (int32_t)roundf(lat * COORD_SCALE);
    int32_t lonI = (int32_t)roundf(lon * COORD_SCALE * lonExtraScale);

    // Извлечение младших 16 бит для каждого компонента
    uint16_t lat16 = (uint16_t)(latI & 0xFFFF);
    uint16_t lon16 = (uint16_t)(lonI & 0xFFFF);

    // Формирование 32-битного слова: Lat (старшие), Lon (младшие)
    return ((uint32_t)lat16 << 16) | lon16;
} // Конец метода pack

void GeoPacker::unpack(uint32_t packed, float myLat, float myLon, 
                       float &outLat, float &outLon) const {
    uint16_t lat16 = (uint16_t)(packed >> 16);
    uint16_t lon16 = (uint16_t)(packed & 0xFFFF);

    int32_t myLatI = (int32_t)roundf(myLat * COORD_SCALE);
    int32_t myLonI = (int32_t)roundf(myLon * COORD_SCALE * lonExtraScale);

    // Восстановление полных целочисленных значений
    int32_t resLatI = recoverComponent(myLatI, lat16);
    int32_t resLonI = recoverComponent(myLonI, lon16);

    // Обратное преобразование во float
    outLat = (float)resLatI / COORD_SCALE;
    outLon = (float)resLonI / (COORD_SCALE * lonExtraScale);
} // Конец метода unpack

float GeoPacker::getLonScale() const {
    return lonExtraScale;
} // Конец метода getLonScale

int32_t GeoPacker::recoverComponent(int32_t referenceFull, uint16_t received16) const {
    // Совмещение старших бит опорного значения с полученными младшими битами
    int32_t candidate = (referenceFull & 0xFFFF0000) | received16;
    int32_t diff = candidate - referenceFull;

    // Коррекция при пересечении границы 16-битного сегмента (32768 единиц ~ 36 км)
    if (diff > 32768) {
        candidate -= 65536;
    } else if (diff < -32768) {
        candidate += 65536;
    } // Конец блока коррекции границ

    return candidate;
} // Конец метода recoverComponent
```

---

## File: .\src\GeoPacker.h
```cpp
#ifndef GEOPACKER_H
#define GEOPACKER_H

#include <stdint.h>

/**
 * Класс для компактной упаковки географических координат в 32-битное целое число.
 * Использует 16 бит для широты и 16 бит для долготы.
 */
class GeoPacker {
public:
    /**
     * Конструктор инициализирует множитель долготы значением по умолчанию (2.0).
     */
    GeoPacker();

    /**
     * Вычисляет множитель долготы на основе широты местности.
     * Округляет значение до сотых для обеспечения синхронизации между устройствами.
     * @param currentLat Текущая широта в градусах.
     */
    void updateLonScale(float currentLat);

    /**
     * Упаковывает float координаты в uint32_t.
     * @param lat Широта.
     * @param lon Долгота.
     * @return Упакованное 32-битное значение.
     */
    uint32_t pack(float lat, float lon) const;

    /**
     * Восстанавливает полные координаты из 16-битных сегментов относительно опорной точки.
     * @param packed Упакованное 32-битное значение.
     * @param myLat Широта опорной точки (текущего устройства).
     * @param myLon Долгота опорной точки (текущего устройства).
     * @param outLat Ссылка для записи восстановленной широты.
     * @param outLon Ссылка для записи восстановленной долготы.
     */
    void unpack(uint32_t packed, float myLat, float myLon, 
                float &outLat, float &outLon) const;

    /**
     * Возвращает текущий множитель долготы.
     */
    float getLonScale() const;

private:
    float lonExtraScale; // Адаптивный множитель для долготы
    static constexpr float COORD_SCALE = 100000.0f; // Базовый множитель (5 знаков)

    /**
     * Внутренний метод для восстановления 32-битной координаты из 16-битного фрагмента.
     */
    int32_t recoverComponent(int32_t referenceFull, uint16_t received16) const;
}; // Конец объявления класса GeoPacker

#endif // GEOPACKER_H
```

---

## File: .\src\GpsManager.cpp
```cpp
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
```

---

## File: .\src\GpsManager.h
```cpp
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
```

---

## File: .\src\logger.h
```cpp
/**
 * Project: Naviga-Dongle
 * File: logger.h
 * Description: Макросы для стандартизированного и читаемого вывода 
 * отладочной информации в Serial Monitor с использованием ANSI-цветов.
 */

 #ifndef LOGGER_H
 #define LOGGER_H
 
 #include <Arduino.h> // Подключение базовой библиотеки Arduino для работы с объектом Serial
 
 // Цветовые коды ANSI
 // Поддерживаются большинством современных терминалов (в т.ч. PlatformIO / VS Code Serial Monitor)
 #define LOG_COLOR_RESET  "\x1b[0m"   // Сброс цвета терминала к стандартному
 #define LOG_COLOR_INFO   "\x1b[32m"  // Зеленый (для успешных операций и стандартных статусов)
 #define LOG_COLOR_WARN   "\x1b[33m"  // Желтый (для предупреждений, коллизий и повторных попыток)
 #define LOG_COLOR_ERROR  "\x1b[31m"  // Красный (для критических ошибок оборудования и инициализации)
 #define LOG_COLOR_DEBUG  "\x1b[36m"  // Циан (для глубокой отладки и трассировки данных)
 
 // Макросы для удобного вывода
 // Форматируют строку в виде: Цвет -> [УРОВЕНЬ][ТЕГ] Текст -> Сброс цвета -> Перенос строки.
 // Поддерживают переменное число аргументов (VA_ARGS) для использования в стиле printf().
 #define LOG_INFO(tag, fmt, ...)  Serial.printf(LOG_COLOR_INFO "[INFO][%s] " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)
 #define LOG_WARN(tag, fmt, ...)  Serial.printf(LOG_COLOR_WARN "[WARN][%s] " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)
 #define LOG_ERROR(tag, fmt, ...) Serial.printf(LOG_COLOR_ERROR "[ERR ][%s] " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)
 
 #endif // LOGGER_H
```

---

## File: .\src\main.cpp
```cpp
/** Видишь?
 * Project: Naviga-Dongle (T-Beam v1.1 Custom E22 + Universal GPS)
 * File: main.cpp
 * Version: 1.9 Изменение: Синхронизация логики handleCollision с адаптивной отправкой координат.
 * Version: 1.8 Изменение: Внедрение адааптивной посылки координат.
 * Description: Главный файл оркестратора.
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <SPI.h>              
 #include "configuration.h"
 #include "logger.h"           
 #include "GeoPacker.h"        
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"     
 #include "Retranslation.h"    
 #include "GpsManager.h"       
 #include "DisplayManager.h"   
 #include "RadioManager.h"     
 #include "PowerManager.h"     
 #include "PacketManager.h"    
 #include "TxManager.h"        

 // --- НАСТРОЙКИ СКАНИРОВАНИЯ ---
 uint32_t networkScanDuration = 30000; 
 #define RED_LED_PIN 4                 

 uint8_t myNodeId = 0; 
 uint8_t myMsgSeq = 0;
 uint8_t myNodeType = NODE_STALKER; // Наша роль по умолчанию

 PowerManager power;                                      
 DisplayManager display(0x3c, I2C_SDA, I2C_SCL); 
 GpsManager gps;                                       
 RadioManager radio; 
 GeoPacker packer;                                      
 NodeDatabase nodeDB;                                   
 Retranslation router;                                  
 PacketManager packetManager(nodeDB, gps, packer);
 TxManager txManager(radio, packer, myNodeId, myMsgSeq);

 volatile bool receivedFlag = false;                    

 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
     receivedFlag = true;
 } // setFlag()

 uint32_t lastTxTime = 0;                               
 uint32_t lastGpsLogTime = 0;                           
 uint32_t lastCleanupTime = 0;      // Таймер для неблокирующей очистки БД                          
 uint32_t lastHeartbeatTime = 0;    // Таймер для фоновой рассылки NodeInfo
 uint32_t lastTopologyUpdateTime = 0;

 bool isLonScaleSet = false;                            

 int getConnectionQuality(uint8_t targetId) {
     if (targetId == myNodeId) return 10; 

     const NodeRecord* target = nodeDB.getNode(targetId);
     if (target == nullptr || targetId == 0) return 0;
     if (millis() - target->lastSeen > 30000) return 0;
     
     if (target->snr <= -99.0f) return 0; 

     int q = map((long)target->snr, -11, 5, 1, 10);
     if (q < 1) q = 1;
     if (q > 10) q = 10;
     return q;
 } // getConnectionQuality()

 void updateScreenCb(String line1, String line2, String line3, String line4) {
     display.showStatus(line1, line2, line3, line4);
 } // updateScreenCb()

 void cycleGpsPowerCb() {
     power.cycleGpsPower();
 } // cycleGpsPowerCb()

 void handleCollision() {
    uint8_t oldId = myNodeId;
    nodeDB.removeNode(oldId); 
    
    randomSeed(esp_random());
    do {
        myNodeId = random(1, 255);
    } while (nodeDB.getNode(myNodeId) != nullptr); // do-while
    
    LOG_WARN("COLLISION", "ID %d is taken! Switched to new ID: %d", oldId, myNodeId);
    
    myMsgSeq = 0; 
    
    // Координаты в базу здесь больше не записываем. 
    // Адаптивная логика в loop() сама увидит "новый" ID и отправит GPS-пакет через 5 секунд.
    
    char myName[12];
    snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
    
    // Отправляем критическое оповещение о смене имени
    txManager.sendNodeInfo(myName, myNodeType, TX_CRITICAL);
    
    // Больше не трогаем lastTxTime вручную. 
    // loop() сам отработает задержку TX_INTERVAL_MOVING (5 сек) перед посылкой координат.
 } // handleCollision()

 void scanNetworkForUniqueId() {
     LOG_INFO("SYS", "Starting network scan for %d ms...", networkScanDuration);
     
     pinMode(RED_LED_PIN, OUTPUT);
     digitalWrite(RED_LED_PIN, LOW); 
     
     uint32_t scanStart = millis();
     uint32_t lastDispUpdate = 0;
     
     receivedFlag = false;
     radio.startReceive(); 
     
     while (millis() - scanStart < networkScanDuration) {
         uint32_t now = millis();
         
         if (now - lastDispUpdate >= 1000) {
             lastDispUpdate = now;
             uint32_t left = (networkScanDuration - (now - scanStart)) / 1000;
             display.showStatus("Scanning Net...", "Time left: " + String(left) + " s", "Nodes Found: " + String(nodeDB.getActiveNodesCount()), "Please wait...");
         } // if (now - lastDispUpdate >= 1000)
         
         if (receivedFlag) {
             noInterrupts(); receivedFlag = false; interrupts();
             
             size_t len = radio.getPacketLength();
             if (len >= sizeof(NavigaHeader)) {
                 uint8_t rxBuffer[256];             
                 int state = radio.readData(rxBuffer, len); 
                 if (state == RADIOLIB_ERR_NONE) {
                     NavigaHeader rxHeader;
                     memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                     size_t payloadLen = len - sizeof(NavigaHeader);
                     
                     if (router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         if (!router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                             packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));
                         } // if (!router.isDuplicate(...))
                     } // if (router.isValidPacket(...))
                 } // if (state == RADIOLIB_ERR_NONE)
             } // if (len >= sizeof(NavigaHeader))
             radio.startReceive();
         } // if (receivedFlag)
         
         gps.update(); 
     } // while (millis() - scanStart < networkScanDuration)
     
     digitalWrite(RED_LED_PIN, HIGH); 
     
     randomSeed(esp_random());
     do {
         myNodeId = random(1, 255);
     } while (nodeDB.getNode(myNodeId) != nullptr); // do-while
     
     LOG_INFO("SYS", "Scan complete. Selected unique Node ID: %d", myNodeId);
     display.showStatus("Scan Complete", "My new ID:", String(myNodeId), "Starting...");
     delay(2000);
 } // scanNetworkForUniqueId()

 void setup() {
     delay(500); 
     Serial.begin(115200);
     unsigned long start = millis();
     while (!Serial && (millis() - start < 3000)); // while
     LOG_INFO("SYS", "--- DONGLE BOOT START ---");
     pinMode(LORA_ONBOARD_CS, OUTPUT);
     digitalWrite(LORA_ONBOARD_CS, HIGH);
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF); 
     Wire.begin(I2C_SDA, I2C_SCL);                       
     SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
     
     power.init();
     display.init(); 
     display.showLogo();
     gps.init(updateScreenCb, cycleGpsPowerCb); 
     
     display.showStatus("System Init...", "GPS Init Done", "Init LoRa...", "");
     if (!radio.init(setFlag)) {
         display.showStatus("ERROR", "LoRa Init Failed", "Check Logs", "");
         delay(3000);
     } // if (!radio.init(...))

     scanNetworkForUniqueId();
     
     char myName[12];
     snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
     txManager.sendNodeInfo(myName, myNodeType, TX_NORMAL);
     
     lastTxTime = millis(); 
 } // setup()
 
 void loop() {

    // --- Очистка устаревших узлов (раз в 10 секунд) ---
    uint32_t currentMillis = millis();

    // --- Обновление топологии по таймеру ---
    if (currentMillis - lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS) {
        nodeDB.updateTopology();
        lastTopologyUpdateTime = currentMillis;
    }

    // --- Cleanup базы по таймеру ---
    if (currentMillis - lastCleanupTime > CLEANUP_INTERVAL_MS) {
        nodeDB.cleanup();
        lastCleanupTime = currentMillis;
    } // if (currentMillis - lastCleanupTime > 10000)

    // --- Редкий фоновый пинг NodeInfo (Heartbeat) ---
    if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
        if (myNodeId != 0) { // Отправляем только если ID уже проинициализирован
            char currentName[12];
            snprintf(currentName, sizeof(currentName), "Node-%d", myNodeId);
            
            // Отправляем пакет с приоритетом TX_NORMAL
            txManager.sendNodeInfo(currentName, myNodeType, TX_NORMAL);
            
            // Логируем существенное действие системы
            LOG_INFO("ACTION", "Heartbeat sent: NodeInfo (Name: %s, Type: %d)", currentName, myNodeType);
        } // if (myNodeId != 0)
        
        lastHeartbeatTime = currentMillis;
    } // if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS)

     gps.update();

     if (receivedFlag) {
         noInterrupts(); receivedFlag = false; interrupts();
         
         size_t len = radio.getPacketLength();
         if (len > 0) {
             uint8_t rxBuffer[256];             
             int state = radio.readData(rxBuffer, len); 
             if (state == RADIOLIB_ERR_NONE) {
                 
                 float currentSNR = radio.getSNR();
                 
                 if (len >= sizeof(NavigaHeader)) {
                     NavigaHeader rxHeader;
                     memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                     size_t payloadLen = len - sizeof(NavigaHeader);
                     
                     bool isCollision = false;
                     bool isOwnEcho = false;

                     if (rxHeader.relayId == myNodeId) {
                         isCollision = true;
                         LOG_WARN("LORA", "Collision Type 1: Relay ID == myNodeId!");
                     } else if (rxHeader.senderId == myNodeId) {
                         int8_t seqDiff = (int8_t)(myMsgSeq - rxHeader.msgSeq);
                         if (seqDiff <= 0 || seqDiff > 10) {
                             isCollision = true;
                             LOG_WARN("LORA", "Collision Type 2: Seq %d vs my %d (diff: %d)", rxHeader.msgSeq, myMsgSeq, seqDiff);
                         } else {
                             LOG_INFO("LORA", "Valid echo of our pkt Seq %d", rxHeader.msgSeq);
                             isOwnEcho = true;
                         } // if (seqDiff <= 0 || ...)
                     } // if (rxHeader.relayId == myNodeId)

                     if (isCollision) {
                         handleCollision();
                     } // if (isCollision)

                     if (rxHeader.relayId != myNodeId) {
                         nodeDB.updateNodeSNR(rxHeader.relayId, currentSNR);
                     } // if (rxHeader.relayId != myNodeId)

                     if (isOwnEcho) {
                         // do nothing
                     } else if (!router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d", rxHeader.getType(), payloadLen);
                     } else if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         LOG_WARN("LORA", "Duplicate pkt Node %d Seq %d dropped.", rxHeader.senderId, rxHeader.msgSeq);
                         txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                     } else {
                         LOG_INFO("LORA", "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, SNR: %.1f)", 
                                  rxHeader.getType(), rxHeader.senderId, rxHeader.relayId, rxHeader.msgSeq, currentSNR);
                         
                            // НОВОЕ: 1. Проверяем, новый ли это узел (до его активации в базе)
                            bool isNewNode = !nodeDB.isNodeActive(rxHeader.senderId);
                                                    
                            // 2. Штатная обработка пакета (здесь узел добавляется/обновляется)
                            packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader));

                            // НОВОЕ: 3. Реактивное Приветствие (батчинг ответов)
                            if (isNewNode && rxHeader.senderId != myNodeId) {
                                uint32_t currentMillis = millis();
                                uint32_t jitterMs = random(MIN_GREETING_NODEINFO_JITTER, MAX_GREETING_NODEINFO_JITTER); // От 2 до 5 минут (120000 - 300000 мс)
                                // Безусловное "состаривание" таймера Heartbeat:
                                // Сдвигаем lastHeartbeatTime так, чтобы до планового срабатывания осталось ровно jitterMs
                                lastHeartbeatTime = currentMillis - HEARTBEAT_INTERVAL_MS + jitterMs;
                                
                                LOG_INFO("SYS", "New Node %d discovered! NodeInfo reply (batching) scheduled in %d sec", rxHeader.senderId, jitterMs / 1000);
                            } // if (isNewNode && rxHeader.senderId != myNodeId)

                         if (router.shouldRetransmit(rxHeader, nodeDB)) {
                             txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, currentSNR);
                         } // if (router.shouldRetransmit(...))
                     } // if (isOwnEcho)
                 } // if (len >= sizeof(NavigaHeader))
             } // if (state == RADIOLIB_ERR_NONE)
         } // if (len > 0)
         radio.startReceive();
     } // if (receivedFlag)

    // --- АДАПТИВНАЯ ОТПРАВКА КООРДИНАТ ---
    if (gps.isValid()) {
        bool shouldTransmit = false;
        const NodeRecord* myRecord = nodeDB.getNode(myNodeId);
        
        // Получаем дистанцию (рассчитанную в блоке экрана) и текущую скорость
        float distFromLastTx = (myRecord != nullptr) ? myRecord->distance : 0.0f;
        float currentSpeed = gps.getSpeed();
        uint32_t now = millis();

        // Гибридный фильтр движения
        if (distFromLastTx > MIN_MOVEMENT_METERS && currentSpeed > MIN_SPEED_KMPH) {
            shouldTransmit = true;
        } else if (distFromLastTx > SNEAK_MOVEMENT_METERS) {
            shouldTransmit = true;
        } // if (distFromLastTx > ...)

        // Принятие решения об отправке
        if (shouldTransmit && (now - lastTxTime >= TX_INTERVAL_MOVING)) {
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0, false); // Сохраняем ТОЛЬКО при отправке
            LOG_INFO("ACTION", "Adaptive TX (Moving): Dist: %.1fm, Speed: %.1fkm/h", distFromLastTx, currentSpeed);
            lastTxTime = now;
        } else if (now - lastTxTime >= TX_INTERVAL_STILL) {
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0, false);
            LOG_INFO("ACTION", "Adaptive TX (Still Heartbeat)");
            lastTxTime = now;
        } // if (shouldTransmit && ...)
    } else {
        // Если нет FIX, просто сбрасываем таймер STILL, чтобы не спамить лог
        if (millis() - lastTxTime >= TX_INTERVAL_STILL) {
            LOG_WARN("TX", "Skip TX: GPS location not valid.");
            lastTxTime = millis(); 
        } // if (millis() - lastTxTime >= TX_INTERVAL_STILL)
    } // if (gps.isValid())

     txManager.processQueue();

     if (millis() - lastCleanupTime >= NODE_TIMEOUT_MS) {
         lastCleanupTime = millis();
         nodeDB.cleanup();
     } // if (millis() - lastCleanupTime >= ...)

     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
         
         String line1, line2, line3, line4;
         int sats = gps.getSatellites();
 
         uint8_t currentTargetId = packetManager.getLastTargetId();
         const NodeRecord* targetNode = nodeDB.getNode(currentTargetId);
         
         bool isTargetValid = (targetNode != nullptr && targetNode->isActive);

         if (!isTargetValid && currentTargetId != 0) {
             packetManager.clearLastTargetId();
             currentTargetId = 0;
         } // if (!isTargetValid && currentTargetId != 0)

         if (!gps.isValid()) {
             line1 = (sats > 0) ? ("GPS Wait " + String(sats)) : "GPS ERROR";
         } else {
             line1 = "GPS OK " + String(sats);

             nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0, false);
             
             if (!isLonScaleSet) {
                 packer.updateLonScale(gps.getLat());
                 isLonScaleSet = true;
             } // if (!isLonScaleSet)

             for (int i = 1; i < 255; i++) {
                 const NodeRecord* node = nodeDB.getNode(i);
                 if (node != nullptr && node->isActive) {
                     
                     if (node->packedCoords != 0 && node->lat == 0.0f && node->lon == 0.0f) {
                         float unpLat, unpLon;
                         packer.unpack(node->packedCoords, gps.getLat(), gps.getLon(), unpLat, unpLon);
                         nodeDB.updateNodeCoords(i, unpLat, unpLon, node->packedCoords, false);
                     } // if (node->packedCoords != 0 && ...)
                     
                     if (node->lat != 0.0f || node->lon != 0.0f) {
                         float d = gps.distanceTo(node->lat, node->lon);
                         float a = gps.courseTo(node->lat, node->lon);
                         nodeDB.updateNodeDistanceAzimuth(i, d, a);
                     } // if (node->lat != 0.0f || ...)
                 } // if (node != nullptr && node->isActive)
             } // for (int i = 1; i < 255; i++)
         } // if (!gps.isValid())
 
         line2 = "My: " + String(myNodeId) + "-" + String(myMsgSeq);
         
         uint8_t totalNodes = nodeDB.getActiveNodesCount();
         uint8_t neighbors = (totalNodes > 0) ? (totalNodes - 1) : 0;
         line3 = "Neighbors: " + String(neighbors);
         
         if (isTargetValid) {
             line4 = String(targetNode->nodeId) + ": " + 
                     String((int)targetNode->distance) + "m/" + 
                     String((int)targetNode->azimuth) + "/" + 
                     String(getConnectionQuality(targetNode->nodeId));
         } else {
             line4 = "No targets";
         } // if (isTargetValid)

         display.showStatus(line1, line2, line3, line4); 
     } // if (millis() - lastGpsLogTime >= gpsUpdateInterval)
 } // loop()
```

---

## File: .\src\NavigaProtocol.h
```cpp
/**
 * File: NavigaProtocol.h
 * Version: 1.1.2
 * Description: Добавлены типы узлов (Tracker, Stalker, Relay) в структуру NodeInfo.
 * Изменение: Приведение к новым правилам оформления (комментирование скобок).
 */
 #ifndef NAVIGA_PROTOCOL_H
 #define NAVIGA_PROTOCOL_H
 
 #include <Arduino.h>
 
 enum NavigaMessageType : uint8_t {
     MSG_COORDS = 1,
     MSG_NODE_INFO = 2,
     MSG_LEAVE = 3
 }; // enum NavigaMessageType
 
 // НОВОЕ: Типы узлов
 enum NodeType : uint8_t {
     NODE_TRACKER = 0, // Мобильный, приоритетно шлет координаты
     NODE_STALKER = 1, // Мобильный/умеренный, шлет координаты, умно ретранслирует
     NODE_RELAY = 2    // Стационарный, ретранслирует всё
 }; // enum NodeType
 
 struct NavigaHeader {
     uint8_t senderId;
     uint8_t relayId;
     uint8_t msgSeq;
     uint8_t typeAndTTL; 
 
     void setTypeAndTTL(NavigaMessageType type, uint8_t ttl) {
         typeAndTTL = (type << 4) | (ttl & 0x0F);
     } // setTypeAndTTL()
 
     NavigaMessageType getType() const {
         return static_cast<NavigaMessageType>(typeAndTTL >> 4);
     } // getType()
 
     uint8_t getTTL() const {
         return typeAndTTL & 0x0F;
     } // getTTL()
 }; // struct NavigaHeader
 
 struct PayloadCoords {
     uint32_t packedCoords;
 }; // struct PayloadCoords
 
 struct PayloadNodeInfo {
     uint8_t nodeType;
     char nodeName[11]; // Строго 11 байт (+1 байт типа = 12 байт Payload)
 }; // struct PayloadNodeInfo
 
 struct PayloadLeave {
     uint8_t reason;
 }; // struct PayloadLeave
 
 struct MessagePolicy {
     bool isRoutable;
     size_t expectedSize;
 }; // struct MessagePolicy
 
 inline MessagePolicy getMessagePolicy(uint8_t msgType) {
     switch (msgType) {
         case MSG_COORDS:     return {true,  sizeof(PayloadCoords)};
         case MSG_NODE_INFO:  return {true,  sizeof(PayloadNodeInfo)};
         case MSG_LEAVE:      return {false, sizeof(PayloadLeave)};
         default:             return {false, 0};
     } // switch (msgType)
 } // getMessagePolicy()
 
 #endif // NAVIGA_PROTOCOL_H
```

---

## File: .\src\NodeDatabase.cpp
```cpp
/**
 * File: NodeDatabase.cpp
 * Version: 1.12 Изменение: Поддержание счетчика узлов на лету и функция updateTopology.
 * Description: Реализация базы данных узлов.
 */
 #include "NodeDatabase.h"
#include "logger.h"
#include "configuration.h"
#include <string.h>
 
 NodeDatabase::NodeDatabase() {
    _activeNodesCount = 0;
    _cachedMaxDist = 0.0f;
    for (int i = 0; i < MAX_NODES; i++) {
         nodes[i].nodeId = i;
         nodes[i].isActive = false;
         nodes[i].lastSeen = 0;
         nodes[i].packedCoords = 0;
         nodes[i].lat = 0.0f;
         nodes[i].lon = 0.0f;
         nodes[i].snr = -100.0f; 
         nodes[i].distance = 0.0f; 
         nodes[i].azimuth = 0.0f;  
         nodes[i].type = NODE_STALKER; // Дефолтный тип
         snprintf(nodes[i].nodeName, sizeof(nodes[i].nodeName), "Node-%d", i);
     } // for (int i = 0; i < MAX_NODES; i++)
 } // NodeDatabase::NodeDatabase()
 
 const NodeRecord* NodeDatabase::getNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return nullptr;
     
     if (!nodes[nodeId].isActive) {
         nodes[nodeId].isActive = true;
         nodes[nodeId].lastSeen = millis();
         nodes[nodeId].packedCoords = 0;
         nodes[nodeId].lat = 0.0f;
         nodes[nodeId].lon = 0.0f;
         nodes[nodeId].snr = -100.0f; 
         nodes[nodeId].distance = 0.0f;
         nodes[nodeId].azimuth = 0.0f;
         nodes[nodeId].type = NODE_STALKER;
         _activeNodesCount++;            // НОВОЕ: Увеличиваем счетчик при активации
         snprintf(nodes[nodeId].nodeName, sizeof(nodes[nodeId].nodeName), "Node-%d", nodeId);
     } // if (!nodes[nodeId].isActive)
     return &nodes[nodeId];
 } // NodeDatabase::getNode()

// Метод только для чтения статуса
bool NodeDatabase::isNodeActive(uint8_t nodeId) const {
    if (nodeId == 0 || nodeId >= MAX_NODES) return false;
    return nodes[nodeId].isActive;
} // NodeDatabase::isNodeActive()

 void NodeDatabase::updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     getNode(nodeId);
     nodes[nodeId].type = nodeType;
     strncpy(nodes[nodeId].nodeName, name, sizeof(nodes[nodeId].nodeName) - 1);
     nodes[nodeId].nodeName[sizeof(nodes[nodeId].nodeName) - 1] = '\0';
     nodes[nodeId].lastSeen = millis();
 } // NodeDatabase::updateNodeInfo()
 
 void NodeDatabase::updateNodeSNR(uint8_t nodeId, float snr) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     const NodeRecord* node = getNode(nodeId); 
     if (node != nullptr) {
         nodes[nodeId].snr = snr;
         nodes[nodeId].lastSeen = millis(); 
     } // if (node != nullptr)
 } // NodeDatabase::updateNodeSNR()
 
 void NodeDatabase::updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     const NodeRecord* node = getNode(nodeId);
     if (node != nullptr) {
         nodes[nodeId].distance = dist;
         nodes[nodeId].azimuth = azmt;
     } // if (node != nullptr)
 } // NodeDatabase::updateNodeDistanceAzimuth()
 
 void NodeDatabase::updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     getNode(nodeId); 
     nodes[nodeId].lat = lat;
     nodes[nodeId].lon = lon;
     if (packed != 0) {
         nodes[nodeId].packedCoords = packed;
     } // if (packed != 0)
     if (updateTimer) {
         nodes[nodeId].lastSeen = millis();
     } // if (updateTimer)
 } // NodeDatabase::updateNodeCoords()
 
 void NodeDatabase::removeNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     nodes[nodeId].isActive = false;
     if (_activeNodesCount > 0) _activeNodesCount--; // НОВОЕ: Уменьшаем счетчик при удалении
    } // NodeDatabase::removeNode()
 
    void NodeDatabase::cleanup() {
        uint32_t currentMillis = millis();
        for (int i = 1; i < MAX_NODES; i++) {
            if (nodes[i].isActive && (currentMillis - nodes[i].lastSeen > NODE_TIMEOUT_MS)) {
                nodes[i].isActive = false;
                // НОВОЕ: Уменьшаем счетчик при очистке по таймауту
                if (_activeNodesCount > 0) _activeNodesCount--;
                LOG_INFO("SYS", "Node %d removed by timeout", i);
            }
        }
    } // NodeDatabase::cleanup()
 
// --- Быстрые геттеры из кэша (O(1)) ---

uint8_t NodeDatabase::getActiveNodesCount() const {
    return _activeNodesCount;
}

float NodeDatabase::getCachedMaxDist() const {
    return _cachedMaxDist;
}

// НОВОЕ: Фоновая синхронизация и сбор географии
void NodeDatabase::updateTopology() {
    uint8_t count = 0;
    float maxD = 0.0f;

    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            count++;
            if (nodes[i].distance > maxD) {
                maxD = nodes[i].distance;
            }
        }
    }
    
    // Синхронизируем счетчик (лечение возможных расхождений)
    _activeNodesCount = count;
    _cachedMaxDist = maxD;
    
    LOG_INFO("SYS", "Topology sync: Nodes: %d, MaxDist: %.1fm", _activeNodesCount, _cachedMaxDist);
}
```

---

## File: .\src\NodeDatabase.h
```cpp
/**
 * File: NodeDatabase.h
 * Version: 1.12 Изменение: Добавлено кэширование метрик топологии.
 * Description: Заголовочный файл базы данных узлов.
 */
 #ifndef NODE_DATABASE_H
 #define NODE_DATABASE_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 
 #define MAX_NODES 255

 // Таймаут неактивности узла: 3 часа (3 * 60 * 60 * 1000 = 10800000 мс)
 #define NODE_TIMEOUT_MS 10800000 
 
 struct NodeRecord {
     uint8_t nodeId;
     uint8_t type;       // Тип узла
     char nodeName[12];  // Соразмерно Payload (с запасом под \0)
     float lat;
     float lon;
     uint32_t packedCoords;
     uint32_t lastSeen;
     bool isActive;
     float snr;
     float distance; 
     float azimuth;  
 }; // struct NodeRecord
 
 class NodeDatabase {
    public:
        NodeDatabase();
    
     const NodeRecord* getNode(uint8_t nodeId);
        
     bool isNodeActive(uint8_t nodeId) const;       // Безопасная проверка активности узла (без побочных эффектов)
    
     void updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer = true);
     void updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType);
     void updateNodeSNR(uint8_t nodeId, float snr);
     void updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt);
 
     void removeNode(uint8_t nodeId);
     void cleanup();
     uint8_t getActiveNodesCount() const;
    // Методы топологии
    void updateTopology();
    float getCachedMaxDist() const;
    
 private:
     NodeRecord nodes[MAX_NODES];

     // НОВОЕ: Кэшированные значения
    uint8_t _activeNodesCount;
    float _cachedMaxDist;
    
 }; // class NodeDatabase
 
 #endif // NODE_DATABASE_H
```

---

## File: .\src\PacketManager.cpp
```cpp
/**
 * File: PacketManager.cpp
 * Version: 1.1.1
 * Description: Реализация диспетчера пакетов.
 * Изменение: Приведение к новым правилам оформления (комментирование скобок).
 */
 #include "PacketManager.h"
 #include "logger.h"
 
 PacketManager::PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer)
     : _nodeDB(db), _gps(gps), _packer(packer), _lastTargetId(0) {
 } // PacketManager::PacketManager()
 
 uint8_t PacketManager::getLastTargetId() const {
     return _lastTargetId;
 } // PacketManager::getLastTargetId()
 
 void PacketManager::clearLastTargetId() {
     _lastTargetId = 0;
 } // PacketManager::clearLastTargetId()
 
 void PacketManager::handleCoordsPacket(uint8_t senderId, const uint8_t* payload) {
     uint32_t packedCoords;
     memcpy(&packedCoords, payload, sizeof(PayloadCoords));
 
     if (!_gps.isValid()) {
         LOG_WARN("DISPATCH", "No GPS fix. Saved RAW coords for Node %d", senderId);
         _nodeDB.updateNodeCoords(senderId, 0.0f, 0.0f, packedCoords);
         _lastTargetId = senderId; 
         return;
     } // if (!_gps.isValid())
 
     float unpLat, unpLon;
     _packer.unpack(packedCoords, _gps.getLat(), _gps.getLon(), unpLat, unpLon);
     
     _nodeDB.updateNodeCoords(senderId, unpLat, unpLon, packedCoords);
     _lastTargetId = senderId; 
     LOG_INFO("DISPATCH", "Extracted COORDS: Node %d -> Lat: %.6f, Lon: %.6f", senderId, unpLat, unpLon);
 } // PacketManager::handleCoordsPacket()
 
 void PacketManager::handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload) {
     PayloadNodeInfo info;
     memcpy(&info, payload, sizeof(PayloadNodeInfo));
     
     _nodeDB.updateNodeInfo(senderId, info.nodeName, info.nodeType);
     LOG_INFO("DISPATCH", "Updated Node %d: Name=%s, Type=%d", senderId, info.nodeName, info.nodeType);
 } // PacketManager::handleNodeInfoPacket()
 
 void PacketManager::handleLeavePacket(uint8_t senderId, const uint8_t* payload) {
     PayloadLeave info;
     memcpy(&info, payload, sizeof(PayloadLeave)); 
     _nodeDB.removeNode(senderId);
     LOG_INFO("DISPATCH", "Node %d left the network. Reason code: %d", senderId, info.reason);
 } // PacketManager::handleLeavePacket()
 
 void PacketManager::processPacket(const NavigaHeader& header, const uint8_t* payload) {
     switch(header.getType()) {
         case MSG_COORDS:
             handleCoordsPacket(header.senderId, payload);
             break;
         case MSG_NODE_INFO:
             handleNodeInfoPacket(header.senderId, payload);
             break;
         case MSG_LEAVE:
             handleLeavePacket(header.senderId, payload);
             break;
         default:
             break;
     } // switch(header.getType())
 } // PacketManager::processPacket()
```

---

## File: .\src\PacketManager.h
```cpp
/**
 * File: PacketManager.h
 * Version: 1.0.0
 * Description: Изолированный класс для парсинга и маршрутизации входящих пакетов.
 */
 #ifndef PACKET_MANAGER_H
 #define PACKET_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"
 #include "GpsManager.h"
 #include "GeoPacker.h"
 
 class PacketManager {
 public:
     // Конструктор принимает ссылки на необходимые подсистемы
     PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer);
 
     // Главный метод обработки пакета (вызывается из loop)
     void processPacket(const NavigaHeader& header, const uint8_t* payload);
     
     // Геттер и сеттер для ID последней цели (для отображения на экране)
     uint8_t getLastTargetId() const;
     void clearLastTargetId();
 
 private:
     NodeDatabase& _nodeDB;
     GpsManager& _gps;
     GeoPacker& _packer;
     uint8_t _lastTargetId;
 
     // Скрытые методы для обработки конкретных типов пакетов
     void handleCoordsPacket(uint8_t senderId, const uint8_t* payload);
     void handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload);
     void handleLeavePacket(uint8_t senderId, const uint8_t* payload);
 };
 
 #endif // PACKET_MANAGER_H
```

---

## File: .\src\PowerManager.cpp
```cpp
/**
 * File: PowerManager.cpp
 * Version: 1.0.0
 * Description: Реализация класса управления питанием.
 */
 #include "PowerManager.h"
 #include "logger.h"
 
 PowerManager::PowerManager() {
     // Конструктор пока пуст, вся работа в init()
 }
 
 bool PowerManager::init() {
     // AXP2101 сидит на той же шине I2C, что и дисплей
     if (_pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
         LOG_INFO("PWR", "AXP2101 PMU initialized successfully.");
         
         // Включаем питание LoRa (ALDO2)
         _pmu.setALDO2Voltage(3300); 
         _pmu.enableALDO2();
         
         // Включаем питание GPS (ALDO3)
         _pmu.setALDO3Voltage(3300); 
         _pmu.enableALDO3();
         
         // Отключаем неиспользуемое
         _pmu.disableALDO4(); 
         
         // Включаем АЦП для замера батареи в будущем
         _pmu.enableSystemVoltageMeasure();  
         return true;
     }
     
     LOG_WARN("PWR", "AXP2101 PMU initialization FAILED!");
     return false;
 }
 
 void PowerManager::cycleGpsPower() {
     LOG_INFO("PWR", "Cycling GPS power (ALDO3)...");
     _pmu.disableALDO3(); 
     delay(2000); 
     _pmu.enableALDO3(); 
     delay(2000); 
     LOG_INFO("PWR", "GPS power restored.");
 }
```

---

## File: .\src\PowerManager.h
```cpp
/**
 * File: PowerManager.h
 * Version: 1.0.0
 * Description: Изолированный класс для управления чипом питания AXP2101.
 */
 #ifndef POWER_MANAGER_H
 #define POWER_MANAGER_H
 
 #include <Arduino.h>
 #include <Wire.h>
 #include <XPowersLib.h>
 #include "configuration.h"
 
 class PowerManager {
 public:
     PowerManager();
 
     // Инициализация чипа и включение нужных линий питания
     bool init();
 
     // Сброс питания на линии ALDO3 (используется для спасения зависшего GPS)
     void cycleGpsPower();
 
     // Задел на будущее: получение уровня заряда батареи
     // int getBatteryLevel();
 
 private:
     XPowersAXP2101 _pmu;
 };
 
 #endif // POWER_MANAGER_H
```

---

## File: .\src\RadioManager.cpp
```cpp
/**
 * File: RadioManager.cpp
 * Version: 1.0.0
 * Description: Реализация класса управления LoRa.
 */
 #include "RadioManager.h"

 // Конструктор: динамически выделяем память под Module и передаем его в SX1268
 RadioManager::RadioManager() : 
     _mod(new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY)), 
     _radio(_mod) {
 }
 
 bool RadioManager::init(void (*isr)(void)) {
     int state = _radio.begin(433.0);
     if (state == RADIOLIB_ERR_NONE) {
         _radio.setDio2AsRfSwitch(false);    
         _radio.setBandwidth(125.0);         
         _radio.setSpreadingFactor(9);       
         _radio.setCodingRate(5);            
         _radio.setSyncWord(0x2B);           
         _radio.setPreambleLength(16);       
         _radio.setOutputPower(22);          
         _radio.setCurrentLimit(140);        
         _radio.setTCXO(1.8);
         _radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         _radio.setRxBoostedGainMode(true);
         _radio.setPacketReceivedAction(isr); // Подключаем аппаратное прерывание
         _radio.startReceive();
         return true;
     }
     return false;
 }
 
 void RadioManager::startReceive() { _radio.startReceive(); }
 void RadioManager::standby() { _radio.standby(); }
 size_t RadioManager::getPacketLength() { return _radio.getPacketLength(); }
 int RadioManager::readData(uint8_t* buffer, size_t len) { return _radio.readData(buffer, len); }
 int RadioManager::transmit(uint8_t* buffer, size_t len) { return _radio.transmit(buffer, len); }
 float RadioManager::getSNR() { return _radio.getSNR(); }
```

---

## File: .\src\RadioManager.h
```cpp
/**
 * File: RadioManager.h
 * Version: 1.0.0
 * Description: Изолированный класс для управления радиомодулем LoRa (SX1268).
 */
 #ifndef RADIO_MANAGER_H
 #define RADIO_MANAGER_H
 
 #include <Arduino.h>
 #include <RadioLib.h>
 #include "configuration.h"
 
 class RadioManager {
 public:
     RadioManager();
 
     // Инициализация. Принимает функцию прерывания (коллбэк). Возвращает true при успехе.
     bool init(void (*isr)(void));
 
     // Проброс базовых методов RadioLib наружу
     void startReceive();
     void standby();
     size_t getPacketLength();
     int readData(uint8_t* buffer, size_t len);
     int transmit(uint8_t* buffer, size_t len);
     float getSNR();
 
 private:
     Module* _mod;
     SX1268 _radio;
 };
 
 #endif // RADIO_MANAGER_H
```

---

## File: .\src\Retranslation.cpp
```cpp
/**
 * File: Retranslation.cpp
 * Version: 1.12 Добавлена оценкf кэшированных расстояний
 * Description: Реализация класса фильтрации эфира.
 */
 #include "Retranslation.h"
#include "NodeDatabase.h" // НОВОЕ: Подключаем базу физически только в .cpp
#include "logger.h"
#include "configuration.h"

 Retranslation::Retranslation() {
     head = 0;
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         history[i].senderId = 0;
         history[i].msgSeq = 0;
     }
 } 
 
 bool Retranslation::isDuplicate(uint8_t senderId, uint8_t msgSeq) {
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         if (history[i].senderId == senderId && history[i].msgSeq == msgSeq) return true; 
     }
     history[head].senderId = senderId;
     history[head].msgSeq = msgSeq;
     head++;
     if (head >= HISTORY_SIZE) head = 0;
     return false; 
 } 
 
 bool Retranslation::isValidPacket(uint8_t msgType, size_t payloadLen) const {
     if (msgType != MSG_NODE_INFO && msgType != MSG_COORDS && msgType != MSG_LEAVE) return false;
     MessagePolicy policy = getMessagePolicy(msgType);
     return (payloadLen == policy.expectedSize);
 } 
 
 bool Retranslation::shouldRetransmit(const NavigaHeader& header, const NodeDatabase& nodeDB) const {
    // Тип пакета - ретранслируемый?
    MessagePolicy policy = getMessagePolicy(header.getType());
     if (!policy.isRoutable) return false;

    // Если TTL пакета исчерпан (остался 1 прыжок, который пакет только что совершил до нас),
    // пакет считается доставленным, но ретранслировать его дальше нельзя.
    if (header.getTTL() <= 1) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: TTL expired.", header.msgSeq);
        return false;
    } // if (header.getTTL() <= 1)
    
    // --- Географические фильтры (мгновенное чтение) ---
    // Фильтр 1: Маленькая сеть
    if (nodeDB.getActiveNodesCount() < 3) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: Network too small (%d nodes).", header.msgSeq, nodeDB.getActiveNodesCount());
        return false;
    }
    // Фильтр 2: Плотная группа (прямая видимость)
    if (nodeDB.getCachedMaxDist() < MAX_DIRECT_CONNECT_METERS) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: Group is dense (Max %.1fm).", header.msgSeq, nodeDB.getCachedMaxDist());
        return false;
    }
    
    return true; 
 } // Retranslation::shouldRetransmit()
```

---

## File: .\src\Retranslation.h
```cpp
/**
 * File: Retranslation.h
 * Version: 1.12 Добавлена база для оценки её кэшированных данных
 * Description: Заголовочный файл класса фильтрации эфира.
 * Теперь содержит ТОЛЬКО логику валидации и анти-дубликатор. 
 * Вся работа с очередями перенесена в TxManager.
 */
 #ifndef RETRANSLATION_H
 #define RETRANSLATION_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 
 // --- НАСТРОЙКИ АНТИ-ДУБЛИКАТОРА ---
 const uint16_t HISTORY_SIZE = 300;
 
 struct PacketRecord {
     uint8_t senderId; 
     uint8_t msgSeq;   
 };
 // НОВОЕ: Forward Declaration (упреждающее объявление)
class NodeDatabase;

 class Retranslation {
 public:
     Retranslation();
 
     // Методы валидации (Таможня)
     bool isDuplicate(uint8_t senderId, uint8_t msgSeq);
     bool isValidPacket(uint8_t msgType, size_t payloadLen) const;
    // ... [подключаем #include "NodeDatabase.h"] ...
    bool shouldRetransmit(const NavigaHeader& header, const NodeDatabase& nodeDB) const;

 private:
     PacketRecord history[HISTORY_SIZE]; 
     uint16_t head;                      
 }; 
 
 #endif // RETRANSLATION_H
```

---

## File: .\src\TxManager.cpp
```cpp
/**
 * File: TxManager.cpp
 * Version: 1.11 Изменение: Уменьшение TTL и перезапись relayId при постановке в очередь ретрансляции.
 * Description: Реализация TxManager.
 */
 #include "TxManager.h"
 #include "logger.h"
 
 extern volatile bool receivedFlag;
 
 TxManager::TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq)
     : _radio(radio), _packer(packer), _myNodeId(nodeId), _myMsgSeq(msgSeq) {
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         _queue[i].isActive = false;
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
     _activeJobIndex = -1;
     _jitterStartTime = 0;
     _jitterDelay = 0;
 } // TxManager::TxManager()
 
 bool TxManager::enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs) {
     if (payloadLen > MAX_PAYLOAD_SIZE) return false;
 
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (!_queue[i].isActive) {
             _queue[i].header = header;
             if (payloadLen > 0 && payload != nullptr) {
                 memcpy(_queue[i].payload, payload, payloadLen);
             } // if (payloadLen > 0 && ...)
             _queue[i].payloadLen = payloadLen;
             _queue[i].priority = priority;
             _queue[i].readyTime = millis() + delayMs;
             _queue[i].isActive = true;
             return true;
         } // if (!_queue[i].isActive)
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
     
     LOG_WARN("TX", "TxQueue is FULL! Dropping packet.");
     return false;
 } // TxManager::enqueue()
 
 void TxManager::sendNodeInfo(const char* nodeName, uint8_t nodeType, TxPriority priority) {
     NavigaHeader txHeader;
     txHeader.senderId = _myNodeId;
     txHeader.relayId = _myNodeId;
     txHeader.msgSeq = _myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_NODE_INFO, DEFAULT_TTL);
 
     PayloadNodeInfo infoPayload;
     infoPayload.nodeType = nodeType; 
     strncpy(infoPayload.nodeName, nodeName, sizeof(infoPayload.nodeName) - 1);
     infoPayload.nodeName[sizeof(infoPayload.nodeName) - 1] = '\0';
 
     enqueue(txHeader, (const uint8_t*)&infoPayload, sizeof(PayloadNodeInfo), priority, 0);
     LOG_INFO("TX", "Enqueued NODE_INFO: Name=%s, Type=%d", nodeName, nodeType);
 } // TxManager::sendNodeInfo()
 
 void TxManager::sendCoords(float lat, float lon, TxPriority priority) {
     NavigaHeader txHeader;
     txHeader.senderId = _myNodeId;
     txHeader.relayId = _myNodeId;
     txHeader.msgSeq = _myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_COORDS, DEFAULT_TTL);
 
     uint32_t packedCoords = _packer.pack(lat, lon);
     enqueue(txHeader, (const uint8_t*)&packedCoords, sizeof(uint32_t), priority, 0);
 } // TxManager::sendCoords()
 
 bool TxManager::enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, float snr) {
    if (payloadLen > MAX_PAYLOAD_SIZE) return false;

    for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
        if (!_queue[i].isActive) {
            _queue[i].header = header;
            
            // НОВОЕ: Логика времени жизни пакета (TTL)
            uint8_t currentTTL = header.getTTL();
            uint8_t newTTL = (currentTTL > 0) ? (currentTTL - 1) : 0;
            _queue[i].header.setTypeAndTTL(header.getType(), newTTL);
            
            // НОВОЕ: Перезаписываем relayId на свой, так как теперь МЫ являемся ретранслятором
            _queue[i].header.relayId = _myNodeId;

            if (payloadLen > 0 && payload != nullptr) {
                memcpy(_queue[i].payload, payload, payloadLen);
            } // if (payloadLen > 0 && payload != nullptr)
            
            _queue[i].payloadLen = payloadLen;
            _queue[i].priority = TX_RELAY;
            _queue[i].rxSnr = snr;
            
            _queue[i].isActive = true;
            return true;
        } // if (!_queue[i].isActive)
    } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
    return false;
} // TxManager::enqueueRelay()

 void TxManager::abortRelay(uint8_t senderId, uint8_t msgSeq) {
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (_queue[i].isActive && _queue[i].header.senderId == senderId && _queue[i].header.msgSeq == msgSeq) {
             _queue[i].isActive = false;
             if (_activeJobIndex == i) {
                 _activeJobIndex = -1;
             } // if (_activeJobIndex == i)
             LOG_INFO("QUEUE", "Relay ABORTED for Seq %d (Suppressed by network)", msgSeq);
             return;
         } // if (_queue[i].isActive && ...)
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
 } // TxManager::abortRelay()
 
 void TxManager::processQueue() {
     if (receivedFlag) {
         _activeJobIndex = -1;
         return;
     } // if (receivedFlag)
 
     uint32_t now = millis();
 
     if (_activeJobIndex == -1) {
         int8_t bestIndex = -1;
         TxPriority bestPriority = TX_RELAY; 
 
         for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
             if (_queue[i].isActive && now >= _queue[i].readyTime) {
                 if (bestIndex == -1 || _queue[i].priority < bestPriority) {
                     bestIndex = i;
                     bestPriority = _queue[i].priority;
                 } // if (bestIndex == -1 || ...)
             } // if (_queue[i].isActive && now >= _queue[i].readyTime)
         } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
 
         if (bestIndex != -1) {
             _activeJobIndex = bestIndex;
             _jitterStartTime = now;
             
             switch(bestPriority) {
                case TX_CRITICAL: _jitterDelay = random(10, 50); break;
                case TX_HIGH:     _jitterDelay = random(50, 150); break;
                case TX_NORMAL:   _jitterDelay = random(100, 300); break;
                
                case TX_RELAY: {
                    // НОВОЕ: Географическая маршрутизация (чем ХУЖЕ сигнал, тем КОРОЧЕ задержка)
                    // Ограничиваем SNR разумными пределами LoRa: от -15 (край зоны) до +5 (рядом)
                    long snrInt = constrain((long)_queue[_activeJobIndex].rxSnr, -15, 5);
                    
                    // Маппинг: -15 dB -> 100 мс (отвечаем первыми), +5 dB -> 1000 мс (ждем)
                    uint32_t baseDelay = map(snrInt, -15, 5, 100, 1000);
                    
                    // Добавляем случайный разброс 0-50 мс для защиты от коллизий равных узлов
                    _jitterDelay = baseDelay + random(0, 50); 
                    break;
                }
            } // switch(bestPriority)
         } // if (bestIndex != -1)
     } // if (_activeJobIndex == -1)
 
     if (_activeJobIndex != -1) {
         if (now - _jitterStartTime >= _jitterDelay) {
             
             uint8_t txBuffer[sizeof(NavigaHeader) + MAX_PAYLOAD_SIZE];
             size_t totalLen = sizeof(NavigaHeader) + _queue[_activeJobIndex].payloadLen;
             
             memcpy(txBuffer, &_queue[_activeJobIndex].header, sizeof(NavigaHeader));
             if (_queue[_activeJobIndex].payloadLen > 0) {
                 memcpy(txBuffer + sizeof(NavigaHeader), _queue[_activeJobIndex].payload, _queue[_activeJobIndex].payloadLen);
             } // if (_queue[_activeJobIndex].payloadLen > 0)
 
             LOG_INFO("TX", "Transmitting Type %d, Seq %d. Priority: %d", 
                      _queue[_activeJobIndex].header.getType(), 
                      _queue[_activeJobIndex].header.msgSeq, 
                      _queue[_activeJobIndex].priority);
 
             _radio.standby();
             _radio.transmit(txBuffer, totalLen);
             receivedFlag = false;
             _radio.startReceive();
 
             _queue[_activeJobIndex].isActive = false; 
             _activeJobIndex = -1; 
         } // if (now - _jitterStartTime >= _jitterDelay)
     } // if (_activeJobIndex != -1)
 } // TxManager::processQueue()
```

---

## File: .\src\TxManager.h
```cpp
/**
 * File: TxManager.h
 * Version: 1.10 Изменение: Добавлено поле rxSnr в TxJob для расчета джиттера при ретрансляции.
 * Description: Единый конвейер для отправки пакетов с поддержкой типа узла.
 */
 #ifndef TX_MANAGER_H
 #define TX_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "RadioManager.h"
 #include "GeoPacker.h"
 
 enum TxPriority {
     TX_CRITICAL = 0, 
     TX_HIGH = 1,     
     TX_NORMAL = 2,   
     TX_RELAY = 3     
 }; // enum TxPriority
 
 const uint8_t TX_QUEUE_SIZE = 15;
 const uint8_t MAX_PAYLOAD_SIZE = 16; 
 
 struct TxJob {
    bool isActive;
    TxPriority priority;
    uint32_t readyTime; 
    NavigaHeader header;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    size_t payloadLen;
    float rxSnr;            //Сохраняем SNR принятого пакета для расчета задержки ретрансляции
 }; // struct TxJob 

 class TxManager {
 public:
     TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq);
 
     void sendNodeInfo(const char* nodeName, uint8_t nodeType, TxPriority priority = TX_NORMAL);
     void sendCoords(float lat, float lon, TxPriority priority = TX_HIGH);
     
     bool enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, float snr);
     void abortRelay(uint8_t senderId, uint8_t msgSeq);
     void processQueue();
 
 private:
     RadioManager& _radio;
     GeoPacker& _packer;
     uint8_t& _myNodeId;
     uint8_t& _myMsgSeq;
 
     TxJob _queue[TX_QUEUE_SIZE];
     
     int8_t _activeJobIndex;
     uint32_t _jitterStartTime;
     uint32_t _jitterDelay;
 
     bool enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs = 0);
 }; // class TxManager
 
 #endif // TX_MANAGER_H
```

---

