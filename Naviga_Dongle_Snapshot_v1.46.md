# Project: Naviga_Dongle Snapshot V1.46

## File: .\src\configuration.h
```cpp
/**
 * File: configuration.h
 * Version: 1.42 
 * Изменение: Добавлен макрос VERSION_STR для централизованного управления версией прошивки.
 * Description: Конфигурация пинов и базовых настроек для разных аппаратных платформ.
 */
 
 #ifndef CONFIGURATION_H
 #define CONFIGURATION_H
 
 #include <Arduino.h>
 
 // ==========================================================
 // --- ВЕРСИЯ ПРОШИВКИ ---
 // ==========================================================
 #define VERSION_STR "1.46"
 
 // ==========================================================
 // --- ВЫБОР ПЛАТЫ (Должен задаваться в platformio.ini) ---
 // ==========================================================
 // Если макрос платы не определен при сборке, по умолчанию используем T-Beam v1.1
 #if !defined(BOARD_T_BEAM_V11) && !defined(BOARD_T_ENERGY_S3)
     #define BOARD_T_BEAM_V11 
 #endif
 
 // ==========================================================
 // СЕКЦИЯ 1: Конфигурация ESP32 T-Beam v1.1 (Оригинал)
 // ==========================================================
 #ifdef BOARD_T_BEAM_V11
     #define BOARD_NAME "T-Beam v1.1"
 
     // Опции аппаратного обеспечения
     #define HAS_PMU 1                // Указывает на наличие чипа управления питанием AXP2101
     #define LORA_CHIP_SX1268         // Используется чип SX1268 (модуль EBYTE E22)
 
     // --- LED (RED) ---
     // Настройки пина светодиода и уровней для включения/выключения
     #define LED_PIN     4   
     #define LED_ON      LOW 
     #define LED_OFF     HIGH 
 
     // --- I2C PINS (OLED & AXP2101) ---
     // Шина I2C используется совместно дисплеем и чипом PMU
     #define I2C_SDA     21      
     #define I2C_SCL     22      
 
     // --- POWER MANAGEMENT (AXP2101) ---
     #define PMU_IRQ     35      // Пин прерывания от контроллера питания
 
     // --- LORA (ONBOARD SX1276) - Отключен аппаратно CS ---
     #define LORA_ONBOARD_CS 18  // Пин CS встроенного (старого) модуля LoRa, который мы глушим
 
     // Отменяем стандартные определения пинов T-Beam из системного pins_arduino.h,
     // чтобы компилятор не выдавал предупреждения "redefined"
     #undef LORA_CS
     #undef LORA_RST
     #undef LORA_IRQ
 
     // --- EBYTE E22-400M33S (SX1268) PINOUT ---
     // Распиновка внешнего, более мощного модуля LoRa
     #define LORA_CS     13  
     #define LORA_RST    14  
     #define LORA_BUSY   15  
     #define LORA_IRQ    35  // DIO1 - Прерывания от LoRa
     #define LORA_TXEN   2   // Управление усилителем передачи (PA)
     #define LORA_RXEN   25  // Управление усилителем приема (LNA)
 
     // --- SPI BUS ---
     #define LORA_SCK    5   
     #define LORA_MISO   19  
     #define LORA_MOSI   27  
 
     // --- GPS (Ublox NEO-8M) ---
     #define GPS_RX      34  
     #define GPS_TX      12  
 #endif
 
 // ==========================================================
 // СЕКЦИЯ 2: Конфигурация Lilygo T-Energy S3 + E22-400M33S + GPS + OLED
 // ==========================================================
 #ifdef BOARD_T_ENERGY_S3
     #define BOARD_NAME "T-Energy S3"
 
     // Опции аппаратного обеспечения
     #define HAS_PMU 0                // Питание прямое, AXP2101 отсутствует
     #define LORA_CHIP_SX1268         
 
     // Стандартный LED для S3 (может варьироваться от ревизии платы)
     #define LED_PIN     0   
     #define LED_ON      HIGH 
     #define LED_OFF     LOW
 
     // --- DISPLAY OLED I2C PINS --- 
     #define I2C_SDA     5 
     #define I2C_SCL     4 
     
     #define PMU_IRQ     -1      // PMU нет, прерываний нет
     #define LORA_ONBOARD_CS -1  // Встроенного LoRa нет
 
     // --- LORA E22-400M33S (SX1268) PINOUT (v1.25) ---
     // Утвержденная распиновка для кастомной сборки на S3
     #define LORA_CS     9   // NSS
     #define LORA_RST    13  // NRST - аппаратный сброс
     #define LORA_BUSY   14  
     #define LORA_IRQ    21  // DIO1
     #define LORA_RXEN   7   // Управление LNA
     #define LORA_TXEN   6   // Управление PA
 
     // --- SPI BUS (Hardware FSPI via GPIO Matrix) ---
     #define LORA_SCK    10  
     #define LORA_MISO   12  
     #define LORA_MOSI   11  
 
     // --- GPS (GY-GPS6MV2 / NEO-6M) ---
     // RX контроллера подключен к TX модуля GPS и наоборот
     #define GPS_RX      44 //18  
     #define GPS_TX      43 //17  
 #endif
 
 // ==========================================================
 // СЕКЦИЯ 3: Глобальные настройки (Общие для всех плат)
 // ==========================================================
 
 // --- ОПЦИИ ИНТЕРФЕЙСА (Рефакторинг UI v1.20) ---
 #define HAS_DISPLAY     1  // 1 - включить код OLED дисплея, 0 - вырезать из компиляции
 #define HAS_STATUS_LED  1  // 1 - включить код статус-светодиода, 0 - вырезать
 
 // --- ТАЙМЕРЫ И ИНТЕРВАЛЫ ---
 const uint32_t txInterval = 10000;          // Базовый интервал передачи (для некоторых подсистем)
 const uint32_t gpsUpdateInterval = 1000;    // Интервал опроса GPS (1 раз в секунду)
 
 #define CLEANUP_INTERVAL_MS 10000           // Как часто запускать сборщик мусора базы узлов
 #define HEARTBEAT_INTERVAL_MS 2700000       // Максимальное время между принудительными отправками статуса
 #define MIN_GREETING_NODEINFO_JITTER 120000 // Мин. задержка перед приветствием нового узла (защита от шторма)
 #define MAX_GREETING_NODEINFO_JITTER 300000 // Макс. задержка перед приветствием
 
 #define TELEMETRY_INTERVAL_MS 10000         // Интервал отправки телеметрии (10 сек)
 
 // --- НАСТРОЙКИ АДАПТИВНОЙ ОТПРАВКИ КООРДИНАТ (v1.18) ---
 #define MIN_MOVEMENT_METERS    15.0f  // Порог дистанции для фиксации движения (защита от дрейфа)
 #define MIN_SPEED_KMPH          2.0f   // Минимальная скорость для подтверждения движения (защита от дрейфа)
 #define SNEAK_MOVEMENT_METERS  40.0f  // Дистанция безусловной отправки (для медленного движения/крадущегося)
 
 // --- НАСТРОЙКИ СЕТИ И РОЛЕЙ ---
 #define DEFAULT_TTL 3                    // Стандартное время жизни пакета (количество прыжков)
 #define MAX_DIRECT_CONNECT_METERS 200.0f // Радиус "компактной группы", внутри которой ретрансляция не нужна
 #define MIN_RELAY_DISTANCE_METERS 20.0f  // Минимальная дистанция до узла для участия в векторной ретрансляции
 #define TOPOLOGY_UPDATE_INTERVAL_MS 15000 // Интервал пересчета сетевой топологии
 #define TRACKER_FAST_SPEED_KMPH 5.0f     // Порог скорости (Бег/Езда) для отключения ретрансляции у Трекера (v1.18)
 
 // --- СТАТИЧЕСКАЯ ПОЗИЦИЯ И ДЖИТТЕР (РОЛИ v1.19) ---
 #define RELAY_STATIC_LAT 0.0f            // Статическая широта для узла-ретранслятора
 #define RELAY_STATIC_LON 0.0f            // Статическая долгота для узла-ретранслятора
 
 // Базовые окна джиттера (задержки) для разных ролей при пересылке пакетов
 #define RELAY_JITTER_MIN_MS 100
 #define RELAY_JITTER_MAX_MS 600
 #define STALKER_JITTER_MIN_MS 300
 #define STALKER_JITTER_MAX_MS 1000
 
 // ==========================================================
 // НАСТРОЙКИ ПО УМОЛЧАНИЮ (При первом запуске) И ЯДРО НАСТРОЕК (v1.27)
 // ==========================================================
 #define DEFAULT_NODE_ID               0
 #define DEFAULT_NODE_TYPE             1            // 1 - NODE_STALKER
 #define DEFAULT_NODE_NAME             "Naviga-Node"
 #define DEFAULT_TX_INTERVAL_MOVING    5000         // Интервал отправки координат в движении (5 сек)
 #define DEFAULT_TX_INTERVAL_STILL     300000       // Интервал отправки Heartbeat на стоянке (5 минут)
 #define DEFAULT_NODE_CONN_TIMEOUT     30000        // 30 сек (потеря активной связи для интерфейса)
 #define DEFAULT_NODE_ACTIVE_TIMEOUT   600000       // 10 минут (очистка узла из локальной базы)
 
 // Структура настроек, хранимая в энергонезависимой памяти (NVS)
 struct NavigaSettings {
     uint8_t nodeId;
     uint8_t nodeType;
     char nodeName[24];              // ИЗМЕНЕНИЕ 1.38: Буфер расширен до 24 байт
     uint32_t txIntervalMoving;
     uint32_t txIntervalStill;
     uint32_t nodeConnectionTimeout; 
     uint32_t nodeActiveTimeoutMs;   
     bool isConfigured;              // Флаг, указывающий, была ли выполнена базовая настройка
 };
 
 #endif // CONFIGURATION_H
```

---

## File: .\src\DisplayManager.cpp
```cpp
/**
 * File: DisplayManager.cpp
 * Version: 1.42 
 * Изменение: Вывод динамической версии из VERSION_STR и увеличение задержки до 3с.
 */
 #include "DisplayManager.h"

 // Конструктор. Инициализирует объект дисплея, если макрос HAS_DISPLAY включен.
 DisplayManager::DisplayManager(uint8_t address, int sda, int scl) 
 #if HAS_DISPLAY
     : _display(address, sda, scl) 
 #endif
 {}
 
 void DisplayManager::init() {
 #if HAS_STATUS_LED
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF); // Выключаем LED при старте
 #endif
 #if HAS_DISPLAY
     _display.init();
     _display.flipScreenVertically(); // Переворачиваем экран для правильной ориентации
 #endif
 }
 
 // Вывод стартового логотипа и информации об инициализации системы
 void DisplayManager::showLogo() {
 #if HAS_DISPLAY
     _display.clear();
     _display.setFont(ArialMT_Plain_16);
     _display.drawString(0, 0,  "Naviga-Dongle");
     
     // ИЗМЕНЕНИЕ 1.42: Использование централизованного макроса VERSION_STR
     String versionLine = "v" + String(VERSION_STR) + " init...";
     _display.drawString(0, 22, versionLine);
     
     _display.drawString(0, 44, "BLE: NimBLE Ready");
     _display.display();
     
     // ИЗМЕНЕНИЕ 1.42: Задержка увеличена до 3 секунд
     delay(5000); 
 #endif
 }
 
 // Универсальный метод вывода 4-х строк текста
 void DisplayManager::showStatus(const String& line1, const String& line2, const String& line3, const String& line4) {
 #if HAS_DISPLAY
     _display.clear();
     _display.setFont(ArialMT_Plain_16);
     _display.drawString(0, 0,  line1);
     _display.drawString(0, 16, line2);
     _display.drawString(0, 32, line3);
     _display.drawString(0, 48, line4);
     _display.display();
 #endif
 }
 
 // Главный метод обновления рабочего экрана. Вызывается регулярно из основного цикла.
 void DisplayManager::updateMainScreen(const char* macSuffix, bool gpsValid, int sats, uint8_t myNodeId, uint8_t myMsgSeq, 
    uint8_t activeNodes, bool hasTarget, uint8_t targetId, 
    int targetDist, int targetAzimuth, int targetQuality,
    BleStatus bleStatus) {
#if HAS_DISPLAY
String line1, line2, line3, line4;

// ИЗМЕНЕНИЕ 1.32: Строка 1 теперь Идентификация (MAC-ID-SEQ)
// Помогает пользователю идентифицировать свой прибор среди других
line1 = String(macSuffix) + "-" + String(myNodeId) + "-" + String(myMsgSeq);

// ИЗМЕНЕНИЕ 1.32: Строка 2 теперь GPS + Статус BLE
// Формируем текстовую метку состояния BLE
String bleLabel = "";
switch(bleStatus) {
case BLE_OFF:       bleLabel = " [-]"; break;
case BLE_UNPAIRED:  bleLabel = " [?]"; break; 
case BLE_CONNECTED: bleLabel = " [+]"; break;
}

// Формируем статус GPS
if (!gpsValid) {
line2 = (sats > 0) ? ("GPS Wait " + String(sats)) : "GPS ERR";
} else {
line2 = "GPS OK " + String(sats);
}
line2 += bleLabel; // Приклеиваем статус BLE к статусу GPS

// Строка 3: Количество активных соседей. Вычитаем себя (1), если база не пуста.
uint8_t neighbors = (activeNodes > 0) ? (activeNodes - 1) : 0;
line3 = "Neighbors: " + String(neighbors);

// Строка 4: Информация о последней цели (соседе, от которого пришел пакет)
if (hasTarget) {
line4 = String(targetId) + ": " + String(targetDist) + "m/" + String(targetAzimuth) + "dg";
} else {
line4 = "No targets";
} 

// Выводим собранные строки на экран
showStatus(line1, line2, line3, line4);
#endif
}
 
 // Переключение состояния светодиода (мигание)
 void DisplayManager::toggleLed() {
 #if HAS_STATUS_LED
     digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
 #endif
 } //DisplayManager.cpp
```

---

## File: .\src\DisplayManager.h
```cpp
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

// Вычисляет масштабный коэффициент для долготы на основе текущей широты.
// Ближе к полюсам расстояние между меридианами сокращается, поэтому коэффициент увеличивается.
void GeoPacker::updateLonScale(float currentLat) {
    float radians = currentLat * (M_PI / 180.0f);
    float cosLat = cosf(radians);

    // Ограничение для исключения деления на ноль вблизи полюсов
    if (cosLat < 0.01f) {
        cosLat = 0.01f;
    } // Конец проверки cosLat

    float rawScale = 1.0f / cosLat;
    
    // Округление множителя до сотых долей для обеспечения строгой
    // математической стабильности при сжатии и распаковке между разными узлами
    lonExtraScale = roundf(rawScale * 100.0f) / 100.0f;
} // Конец метода updateLonScale

// Упаковывает две float-координаты (широту и долготу) в одно 32-битное число
uint32_t GeoPacker::pack(float lat, float lon) const {
    // Умножаем на масштаб (100000) для перевода в целые числа
    int32_t latI = (int32_t)roundf(lat * COORD_SCALE);
    int32_t lonI = (int32_t)roundf(lon * COORD_SCALE * lonExtraScale);

    // Извлечение младших 16 бит для каждого компонента (Дельта-компрессия)
    uint16_t lat16 = (uint16_t)(latI & 0xFFFF);
    uint16_t lon16 = (uint16_t)(lonI & 0xFFFF);

    // Формирование 32-битного слова: Lat (старшие 16 бит), Lon (младшие 16 бит)
    return ((uint32_t)lat16 << 16) | lon16;
} // Конец метода pack

// Распаковывает 32-битную координату обратно во float на основе опорной точки (myLat, myLon)
void GeoPacker::unpack(uint32_t packed, float myLat, float myLon, 
                       float &outLat, float &outLon) const {
    uint16_t lat16 = (uint16_t)(packed >> 16);
    uint16_t lon16 = (uint16_t)(packed & 0xFFFF);

    // Перевод опорных координат в целочисленный формат
    int32_t myLatI = (int32_t)roundf(myLat * COORD_SCALE);
    int32_t myLonI = (int32_t)roundf(myLon * COORD_SCALE * lonExtraScale);

    // Восстановление полных целочисленных значений (объединение базовых старших бит и полученных младших)
    int32_t resLatI = recoverComponent(myLatI, lat16);
    int32_t resLonI = recoverComponent(myLonI, lon16);

    // Обратное преобразование во float
    outLat = (float)resLatI / COORD_SCALE;
    outLon = (float)resLonI / (COORD_SCALE * lonExtraScale);
} // Конец метода unpack

float GeoPacker::getLonScale() const {
    return lonExtraScale;
} // Конец метода getLonScale

// Внутренний метод восстановления 32-битной компоненты из 16-битной
int32_t GeoPacker::recoverComponent(int32_t referenceFull, uint16_t received16) const {
    // Совмещение старших бит опорного значения с полученными младшими битами
    int32_t candidate = (referenceFull & 0xFFFF0000) | received16;
    int32_t diff = candidate - referenceFull;

    // Коррекция при пересечении границы 16-битного сегмента (32768 единиц ~ 36 км)
    // Если разница слишком велика, значит координата перешла границу сегмента, 
    // необходимо сделать математическую поправку.
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
 * Позволяет значительно сэкономить эфирное время (Duty Cycle) LoRa.
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
    static constexpr float COORD_SCALE = 100000.0f; // Базовый множитель (5 знаков после запятой)

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
```

---

## File: .\src\GpsManager.h
```cpp
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
/**
 * Project: Naviga-Dongle
 * File: main.cpp
 * Version: 1.46
 * Изменение: Использование bleManager.sendNodeCoords в Smart TX и
 * bleManager.sendNodeInfo при смене Identity.
 */

#include "BleManager.h"
#include "CoordProcessor.h"
#include "DisplayManager.h"
#include "GeoPacker.h"
#include "GpsManager.h"
#include "NavigaProtocol.h"
#include "NetworkManager.h"
#include "NodeDatabase.h"
#include "PacketManager.h"
#include "PowerManager.h"
#include "RadioManager.h"
#include "Retranslation.h"
#include "SettingsManager.h"
#include "TxManager.h"
#include "configuration.h"
#include "logger.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

uint32_t networkScanDuration = 30000;
uint8_t myNodeId = 0;
uint8_t myMsgSeq = 0;
uint8_t myNodeType = NODE_RELAY;

PowerManager power;
DisplayManager display(0x3c, I2C_SDA, I2C_SCL);
GpsManager gps;
RadioManager radio;
GeoPacker packer;
NodeDatabase nodeDB;
Retranslation router;
BleManager bleManager;
PacketManager packetManager(nodeDB, gps, packer, bleManager);
TxManager txManager(radio, packer, myNodeId, myMsgSeq);
NetworkManager networkManager(radio, nodeDB, display, gps, txManager,
                              bleManager, router, packetManager);
CoordProcessor coordProcessor;

volatile bool receivedFlag = false;
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) { receivedFlag = true; }

uint32_t lastTxTime = 0;
uint32_t lastGpsLogTime = 0;
uint32_t lastCleanupTime = 0;
uint32_t lastHeartbeatTime = 0;
uint32_t lastTopologyUpdateTime = 0;
uint32_t lastTelemetryTime = 0;

void updateScreenCb(String l1, String l2, String l3, String l4) {
  display.showStatus(l1, l2, l3, l4);
}
void cycleGpsPowerCb() { power.cycleGpsPower(); }

void setup() {
  delay(500);
  Serial.begin(115200);
  LOG_INFO("SYS", "--- DONGLE BOOT START v1.46 ---");
#ifdef BOARD_T_BEAM_V11
  pinMode(LORA_ONBOARD_CS, OUTPUT);
  digitalWrite(LORA_ONBOARD_CS, HIGH);
#endif
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(20);
  digitalWrite(LORA_RST, HIGH);
  delay(50);
  Wire.begin(I2C_SDA, I2C_SCL);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  power.init();
  display.init();
  display.showLogo();
  gps.init(updateScreenCb, cycleGpsPowerCb);
  bleManager.init();
  settingsManager.init();
  myNodeId = settingsManager.settings.nodeId;
  myNodeType = settingsManager.settings.nodeType;
  settingsManager.loadNodesSnapshot(nodeDB);
  nodeDB.ageAllNodes(settingsManager.settings.nodeConnectionTimeout + 1000);
  char myName[24];
  strncpy(myName, settingsManager.settings.nodeName, 23);
  myName[23] = '\0';
  nodeDB.addNode(myNodeId);
  nodeDB.updateNodeInfo(myNodeId, myName, myNodeType);
  if (myNodeType == NODE_RELAY)
    gps.setStaticLocation(RELAY_STATIC_LAT, RELAY_STATIC_LON);
  radio.init(setFlag);
  if (myNodeId == 0 || !settingsManager.settings.isConfigured) {
    networkManager.scanNetwork(false, networkScanDuration, myNodeId);
    settingsManager.settings.nodeId = myNodeId;
    settingsManager.settings.isConfigured = true;
    settingsManager.save();
  } else
    networkManager.scanNetwork(true, networkScanDuration, myNodeId);
  txManager.sendNodeInfo(myName, myNodeType, TX_NORMAL);
  lastTxTime = millis();
}

void loop() {
  uint32_t currentMillis = millis();
  float currentSpeed = gps.getSpeed();

  if (bleManager.requestIdentitySync) {
    bleManager.requestIdentitySync = false;
    bleManager.sendIdentity(settingsManager.settings.nodeId,
                            settingsManager.settings.nodeName,
                            settingsManager.settings.nodeType);
  }
  if (bleManager.requestSysConfigSync) {
    bleManager.requestSysConfigSync = false;
    bleManager.sendSysConfig(settingsManager.settings.txIntervalMoving,
                             settingsManager.settings.txIntervalStill,
                             settingsManager.settings.nodeConnectionTimeout,
                             settingsManager.settings.nodeActiveTimeoutMs);
  }
  if (bleManager.requestFullSync) {
    bleManager.requestFullSync = false;
    for (int i = 1; i < 255; i++) {
      const NodeRecord *node = nodeDB.getNode(i);
      if (node != nullptr && node->isActive) {
        BleEvtNodeUpdate update;
        update.opCode = EVT_NODE_UPDATE;
        update.nodeId = node->nodeId;
        update.nodeRole = node->type;
        strncpy(update.nodeName, node->nodeName, 23);
        update.nodeName[23] = '\0';
        update.lat = node->lat;
        update.lon = node->lon;
        update.snr = node->snr;
        update.lastSeenAge = millis() - node->lastSeen;
        bleManager.sendNodeUpdate(update);
        delay(5);
      }
    }
  }
  if (bleManager.hasNewIdentity) {
    bleManager.hasNewIdentity = false;
    myNodeId = bleManager.newIdentity.myNodeId;
    myNodeType = bleManager.newIdentity.myRole;
    settingsManager.settings.nodeId = myNodeId;
    settingsManager.settings.nodeType = myNodeType;
    strncpy(settingsManager.settings.nodeName, bleManager.newIdentity.myName,
            23);
    settingsManager.save();
    txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType,
                           TX_CRITICAL);
    nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName,
                          myNodeType);
    // ИЗМЕНЕНИЕ 1.46: Немедленный частичный апдейт инфо по BLE
    bleManager.sendNodeInfo(myNodeId, myNodeType,
                            settingsManager.settings.nodeName);
  }
  if (bleManager.hasNewSysConfig) {
    bleManager.hasNewSysConfig = false;
    settingsManager.settings.txIntervalMoving =
        bleManager.newSysConfig.txIntervalMoving;
    settingsManager.settings.txIntervalStill =
        bleManager.newSysConfig.txIntervalStill;
    settingsManager.settings.nodeConnectionTimeout =
        bleManager.newSysConfig.nodeConnectionTimeout;
    settingsManager.settings.nodeActiveTimeoutMs =
        bleManager.newSysConfig.nodeActiveTimeoutMs;
    settingsManager.save();
  }
  if (bleManager.requestClearDB) {
    bleManager.requestClearDB = false;
    for (int i = 1; i < 255; i++) {
      if (i != myNodeId)
        nodeDB.removeNode(i);
    }
    settingsManager.saveNodesSnapshot(nodeDB);
  }
  if (bleManager.requestReset) {
    settingsManager.factoryReset();
    delay(500);
    ESP.restart();
  }

  bool isFastTracker =
      (myNodeType == NODE_TRACKER && currentSpeed > TRACKER_FAST_SPEED_KMPH);
  if (gps.isValid() &&
      (currentMillis - lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS)) {
    if (!isFastTracker)
      nodeDB.updateTopology();
    lastTopologyUpdateTime = currentMillis;
  }
  if (currentMillis - lastCleanupTime > CLEANUP_INTERVAL_MS) {
    nodeDB.cleanup(myNodeId);
    lastCleanupTime = currentMillis;
  }
  if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
    if (myNodeId != 0) {
      txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType,
                             TX_NORMAL);
      nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName,
                            myNodeType);
    }
    lastHeartbeatTime = currentMillis;
  }
  gps.update();
  if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = currentMillis;
    if (bleManager.getBleStatus() == BLE_CONNECTED)
      bleManager.sendMyStatus(gps.isValid(), gps.getSatellites(),
                              power.getBatteryPercent(),
                              power.getBatteryVoltage());
  }
  if (receivedFlag) {
    noInterrupts();
    receivedFlag = false;
    interrupts();
    size_t len = radio.getPacketLength();
    if (len >= sizeof(NavigaHeader)) {
      uint8_t rxBuffer[256];
      if (radio.readData(rxBuffer, len) == RADIOLIB_ERR_NONE) {
        float currentSNR = radio.getSNR();
        NavigaHeader rxHeader;
        memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
        if (rxHeader.relayId == myNodeId ||
            (rxHeader.senderId == myNodeId &&
             abs((int8_t)(myMsgSeq - rxHeader.msgSeq)) > 10))
          networkManager.handleCollision(myNodeId, myMsgSeq, myNodeType);
        if (rxHeader.relayId != myNodeId)
          nodeDB.updateNodeSNR(rxHeader.relayId, currentSNR);
        if (router.isValidPacket(rxHeader.getType(),
                                 len - sizeof(NavigaHeader))) {
          if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
            if (!nodeDB.hasNodesInOppositeDirection(rxHeader.relayId))
              txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
          } else {
            bool isNew = !nodeDB.isNodeActive(rxHeader.senderId);
            packetManager.processPacket(rxHeader,
                                        rxBuffer + sizeof(NavigaHeader),
                                        len - sizeof(NavigaHeader));
            if (isNew && rxHeader.senderId != myNodeId) {
              lastHeartbeatTime =
                  millis() - HEARTBEAT_INTERVAL_MS + random(120000, 300000);
              settingsManager.saveNodesSnapshot(nodeDB);
            }
            if (router.shouldRetransmit(rxHeader, nodeDB, myNodeType,
                                        currentSpeed)) {
              uint8_t sR = NODE_STALKER;
              const NodeRecord *sn = nodeDB.getNode(rxHeader.senderId);
              if (sn)
                sR = sn->type;
              txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader),
                                     len - sizeof(NavigaHeader),
                                     networkManager.calculateRelayJitter(
                                         myNodeType, sR, currentSNR));
            }
          }
        }
      }
    }
    radio.startReceive();
  }
  if (gps.isValid()) {
    const NodeRecord *myR = nodeDB.getNode(myNodeId);
    float dLast = (myR != nullptr) ? myR->distance : 0.0f;
    bool shouldTx =
        (dLast > MIN_MOVEMENT_METERS && currentSpeed > MIN_SPEED_KMPH) ||
        (dLast > SNEAK_MOVEMENT_METERS);
    if ((shouldTx && (currentMillis - lastTxTime >=
                      settingsManager.settings.txIntervalMoving)) ||
        (currentMillis - lastTxTime >=
         settingsManager.settings.txIntervalStill)) {
      txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
      nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0);
      // ИЗМЕНЕНИЕ 1.46: Частичный апдейт своих координат по BLE
      if (bleManager.getBleStatus() == BLE_CONNECTED)
        bleManager.sendNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0.0f);
      lastTxTime = currentMillis;
    }
  }
  txManager.processQueue();
  if (currentMillis - lastGpsLogTime >= gpsUpdateInterval) {
    lastGpsLogTime = currentMillis;
    display.toggleLed();
    coordProcessor.process(nodeDB, gps, packer, isFastTracker);
    uint8_t tId = packetManager.getLastTargetId();
    const NodeRecord *tN = nodeDB.getNode(tId);
    bool tV = (tN != nullptr && tN->isActive);
    if (!tV && tId != 0) {
      packetManager.clearLastTargetId();
      tId = 0;
    }
    display.updateMainScreen(
        bleManager.macSuffix, gps.isValid(), gps.getSatellites(), myNodeId,
        myMsgSeq, nodeDB.getActiveNodesCount(), tV, tId,
        tV ? (int)tN->distance : 0, tV ? (int)tN->azimuth : 0,
        tV ? networkManager.getConnectionQuality(tId, myNodeId) : 0,
        bleManager.getBleStatus());
  }
} // main.cpp

```

---

## File: .\src\NavigaProtocol.h
```cpp
/**
 * File: NavigaProtocol.h
 * Version: 1.37
 * Description: Добавлены типы узлов (Tracker, Stalker, Relay) в структуру NodeInfo.
 * Изменение: Переход на пакеты переменной длины и увеличение буфера имени узла до 24 байт (Шаг 1).
 */
 #ifndef NAVIGA_PROTOCOL_H
 #define NAVIGA_PROTOCOL_H
 
 #include <Arduino.h>
 
 // Типы сообщений (NavigaMessageType)
 enum NavigaMessageType : uint8_t {
     MSG_COORDS = 1,     // Пакет с координатами
     MSG_NODE_INFO = 2,  // Пакет с информацией об узле (Имя, Роль)
     MSG_LEAVE = 3       // Сигнал об отключении устройства из сети
 }; // enum NavigaMessageType
 
 // НОВОЕ: Типы узлов (Ролевая модель)
 enum NodeType : uint8_t {
     NODE_TRACKER = 0, // Мобильный, быстро движущийся объект (отключает ретрансляцию, приоритетный TX)
     NODE_STALKER = 1, // Мобильный/умеренный, стандартный участник группы
     NODE_RELAY = 2    // Стационарный ретранслятор (магистраль)
 }; // enum NodeType
 
 // Структура 4-байтного заголовка протокола Naviga (Для упаковки в эфир)
 struct NavigaHeader {
     uint8_t senderId;   // Оригинальный отправитель
     uint8_t relayId;    // Узел, который физически произвел последний прыжок (пересылку)
     uint8_t msgSeq;     // Порядковый номер сообщения отправителя
     uint8_t typeAndTTL; // Комбинированное поле: 4 бита Тип + 4 бита TTL (Time-To-Live)
 
     // Установка комбинированного поля
     void setTypeAndTTL(NavigaMessageType type, uint8_t ttl) {
         typeAndTTL = (type << 4) | (ttl & 0x0F);
     } // setTypeAndTTL()
 
     // Извлечение типа сообщения
     NavigaMessageType getType() const {
         return static_cast<NavigaMessageType>(typeAndTTL >> 4);
     } // getType()
 
     // Извлечение счетчика TTL
     uint8_t getTTL() const {
         return typeAndTTL & 0x0F;
     } // getTTL()
 }; // struct NavigaHeader
 
 // Полезная нагрузка пакета координат (4 байта)
 struct PayloadCoords {
     uint32_t packedCoords; // Сжатые через GeoPacker координаты
 }; // struct PayloadCoords
 
 // Полезная нагрузка пакета NodeInfo (макс 24 байта)
 struct PayloadNodeInfo {
     uint8_t nodeType;
     char nodeName[23]; // ИЗМЕНЕНИЕ 1.37: Расширен до 23 байт (+1 байт типа = макс 24 байта Payload). Завершается нуль-терминатором.
 }; // struct PayloadNodeInfo
 
 // Полезная нагрузка пакета выхода из сети (1 байт)
 struct PayloadLeave {
     uint8_t reason; // Код причины выхода
 }; // struct PayloadLeave
 
 // ИЗМЕНЕНИЕ 1.37: Структура для возврата политики обработки с диапазоном размеров
 struct MessagePolicy {
     bool isRoutable;     // Подлежит ли ретрансляции
     size_t minSize;      // Минимально допустимый размер
     size_t maxSize;      // Максимально допустимый размер
 }; // struct MessagePolicy
 
 // ИЗМЕНЕНИЕ 1.37: Функция маппинга типа сообщения на его политику обработки с диапазонами
 inline MessagePolicy getMessagePolicy(uint8_t msgType) {
     switch (msgType) {
         case MSG_COORDS:     return {true,  sizeof(PayloadCoords), sizeof(PayloadCoords)};
         // MSG_NODE_INFO минимум: 1 байт роли + 1 байт нуль-терминатор (2 байта). Максимум: 24 байта.
         case MSG_NODE_INFO:  return {true,  2,                     sizeof(PayloadNodeInfo)};
         case MSG_LEAVE:      return {false, sizeof(PayloadLeave),  sizeof(PayloadLeave)};
         default:             return {false, 0,                     0};
     } // switch (msgType)
 } // getMessagePolicy()
 
 #endif // NAVIGA_PROTOCOL_H
```

---

## File: .\src\NodeDatabase.cpp
```cpp
/**
 * File: NodeDatabase.cpp
 * Version: 1.27 
 * Изменение: Подключение SettingsManager. Функция cleanup теперь использует динамический таймаут из NVS.
 * Description: Реализация локальной базы данных активных узлов в Mesh-сети.
 */
 #include "NodeDatabase.h"
 #include "logger.h"
 #include "configuration.h"
 #include "SettingsManager.h" // ИЗМЕНЕНИЕ 1.27: Подключение менеджера настроек
 #include <string.h>
 
 NodeDatabase::NodeDatabase() {
     _activeNodesCount = 0;
     _cachedMaxDist = 0.0f;
     for (int i=0; i<4; i++) _quadrantNodes[i] = 0;
     
     // Предварительная инициализация массива из 255 узлов (0-й индекс не используется как активный узел)
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
         nodes[i].type = NODE_STALKER; 
         snprintf(nodes[i].nodeName, sizeof(nodes[i].nodeName), "Node-%d", i);
     } 
 } 
 
 // Получение константного указателя на запись узла (защита от несанкционированного изменения)
 const NodeRecord* NodeDatabase::getNode(uint8_t nodeId) const {
    if (nodeId == 0 || nodeId >= MAX_NODES) return nullptr;
    if (!nodes[nodeId].isActive) return nullptr; 
     return &nodes[nodeId];
 } 

 // Добавление нового узла в базу
 void NodeDatabase::addNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
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
         _activeNodesCount++;            
         snprintf(nodes[nodeId].nodeName, sizeof(nodes[nodeId].nodeName), "Node-%d", nodeId);
     } 
 } 

bool NodeDatabase::isNodeActive(uint8_t nodeId) const {
    if (nodeId == 0 || nodeId >= MAX_NODES) return false;
    return nodes[nodeId].isActive;
} 

 // Обновление информации об узле (Role, Name) из пакета NodeInfo
 void NodeDatabase::updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].type = nodeType;
     strncpy(nodes[nodeId].nodeName, name, sizeof(nodes[nodeId].nodeName) - 1);
     nodes[nodeId].nodeName[sizeof(nodes[nodeId].nodeName) - 1] = '\0'; // Гарантированный нуль-терминатор
     nodes[nodeId].lastSeen = millis();
 } 
 
 void NodeDatabase::updateNodeSNR(uint8_t nodeId, float snr) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].snr = snr;
     nodes[nodeId].lastSeen = millis(); 
 } 
 
 void NodeDatabase::updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].distance = dist;
     nodes[nodeId].azimuth = azmt;
 } 
 
 // Обновление координат узла. Параметр updateTimer позволяет обновлять таймер lastSeen 
 // (по умолчанию true, false используется при загрузке слепка из памяти NVS)
 void NodeDatabase::updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].lat = lat;
     nodes[nodeId].lon = lon;
     if (packed != 0) nodes[nodeId].packedCoords = packed;
     if (updateTimer) nodes[nodeId].lastSeen = millis();
 } 
 
 void NodeDatabase::removeNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     nodes[nodeId].isActive = false;
     if (_activeNodesCount > 0) _activeNodesCount--; 
    } 
 
    // Сборщик мусора: деактивирует узлы по таймауту (от которых давно не было вестей)
    void NodeDatabase::cleanup(uint8_t excludeNodeId) {
        uint32_t currentMillis = millis();
        for (int i = 1; i < MAX_NODES; i++) {
            if (i == excludeNodeId) continue; // Защита собственного узла от случайного удаления
            // ИЗМЕНЕНИЕ 1.27: Используем динамический таймаут из NVS (настройки Оператора)
            if (nodes[i].isActive && (currentMillis - nodes[i].lastSeen > settingsManager.settings.nodeActiveTimeoutMs)) {
                nodes[i].isActive = false;
                if (_activeNodesCount > 0) _activeNodesCount--;
                LOG_INFO("SYS", "Node %d removed by timeout", i);
            }
        }
    } 
 
uint8_t NodeDatabase::getActiveNodesCount() const { return _activeNodesCount; }
float NodeDatabase::getCachedMaxDist() const { return _cachedMaxDist; }

// --- ВЕКТОРНЫЙ ФИЛЬТР ---
// Проверяет, есть ли хотя бы один узел в противоположном квадранте относительно узла,
// который переслал нам пакет (referenceNodeId). Если нет, значит мы — географический тупик сети.
bool NodeDatabase::hasNodesInOppositeDirection(uint8_t referenceNodeId) const {
    if (referenceNodeId == 0 || referenceNodeId >= MAX_NODES) return true;
    if (!nodes[referenceNodeId].isActive) return true;
    
    // Если мы не знаем дистанции до отправителя, разрешаем ретрансляцию (Fallback)
    if (nodes[referenceNodeId].distance == 0.0f && nodes[referenceNodeId].lat == 0.0f) return true; 

    // Определяем в каком мы квадранте (0, 1, 2, 3) находится отправитель относительно нас
    int senderQ = (int)(nodes[referenceNodeId].azimuth / 90.0f) % 4;
    if (senderQ < 0) senderQ = 0; 
    
    // Вычисляем противоположный квадрант
    int oppositeQ = (senderQ + 2) % 4;
    
    // Проверяем, есть ли узлы в противоположном квадранте (данные пересчитываются в updateTopology)
    return _quadrantNodes[oppositeQ] > 0;
}

// Тяжеловесная функция пересчета сетевой топологии
void NodeDatabase::updateTopology() {
    uint8_t count = 0;
    float maxD = 0.0f;
    
    // Сбрасываем счетчики квадрантов
    for (int i=0; i<4; i++) _quadrantNodes[i] = 0;

    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            count++;
            // Поиск максимальной дистанции в группе для фильтра "Компактная группа"
            if (nodes[i].distance > maxD) {
                maxD = nodes[i].distance;
            }
            
            // Распределение узлов по квадрантам для векторного фильтра
            // Исключаем узлы, которые находятся слишком близко (< 20м), 
            // так как ошибка азимута на коротких дистанциях слишком велика
            if (nodes[i].distance >= MIN_RELAY_DISTANCE_METERS) {
                int q = (int)(nodes[i].azimuth / 90.0f) % 4;
                if (q >= 0 && q < 4) {
                    _quadrantNodes[q]++;
                }
            }
        }
    }
    
    _activeNodesCount = count;
    _cachedMaxDist = maxD;
    
    LOG_INFO("SYS", "Topology sync: Nodes: %d, MaxDist: %.1fm. Q:[%d,%d,%d,%d]", 
             _activeNodesCount, _cachedMaxDist, _quadrantNodes[0], _quadrantNodes[1], _quadrantNodes[2], _quadrantNodes[3]);
}

// Искусственное состаривание таймеров lastSeen.
// Используется при включении (Warm Start), когда мы загрузили узлы из флеш памяти, 
// но еще не получили от них свежих подтверждений в эфире. Узлы становятся "серыми".
void NodeDatabase::ageAllNodes(uint32_t ageMs) {
    uint32_t currentMillis = millis();
    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            // Защита от математического переполнения (overflow)
            if (currentMillis >= ageMs) {
                nodes[i].lastSeen = currentMillis - ageMs;
            } else {
                nodes[i].lastSeen = 0;
            }
        }
    }
} //NodeDatabase.cpp
```

---

## File: .\src\NodeDatabase.h
```cpp
/**
 * File: NodeDatabase.h
 * Version: 1.38 
 * Изменение: Увеличен размер буфера nodeName до 24 байт (Шаг 2).
 * Description: Заголовочный файл локальной базы данных активных узлов.
 */
 #ifndef NODE_DATABASE_H
 #define NODE_DATABASE_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 
 #define MAX_NODES 255

 // Старый дефолтный таймаут неактивности узла (заменен на динамический в SettingsManager, но оставлен для резерва)
 #define NODE_TIMEOUT_MS 10800000 
 
 // Структура хранения информации об одном активном узле
 struct NodeRecord {
     uint8_t nodeId;
     uint8_t type;       // Тип узла (Роль)
     char nodeName[24];  // ИЗМЕНЕНИЕ 1.38: Увеличен до 24 байт
     float lat;
     float lon;
     uint32_t packedCoords; // Сырые запакованные координаты (до распаковки GeoPacker)
     uint32_t lastSeen;     // Время (millis()) последнего приема пакета от узла
     bool isActive;
     float snr;             // Последний измеренный уровень сигнала от узла
     float distance;        // Рассчитанная дистанция в метрах до локального устройства
     float azimuth;         // Рассчитанный азимут (направление) от локального устройства
 }; // struct NodeRecord
 
 class NodeDatabase {
    public:
        NodeDatabase();
    
    // Получение указателя на константную запись
    const NodeRecord* getNode(uint8_t nodeId) const;

     bool isNodeActive(uint8_t nodeId) const;       
    
     void addNode(uint8_t nodeId);                  
    
     void updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer = true);
     void updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType);
     void updateNodeSNR(uint8_t nodeId, float snr);
     void updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt);
 
     void removeNode(uint8_t nodeId);
     void cleanup(uint8_t excludeNodeId = 0); 
     
     uint8_t getActiveNodesCount() const;
     
    // Методы топологии
    void updateTopology();
    float getCachedMaxDist() const;
    
    // Векторный фильтр
    bool hasNodesInOppositeDirection(uint8_t referenceNodeId) const;

    // Искусственное старение узлов (сдвиг lastSeen в прошлое) для Warm Start
    void ageAllNodes(uint32_t ageMs);

 private:
     NodeRecord nodes[MAX_NODES]; // Статический массив на 255 узлов

     // Кэшированные значения топологии
    uint8_t _activeNodesCount;
    float _cachedMaxDist;
    uint8_t _quadrantNodes[4]; // 0: 0-90, 1: 90-180, 2: 180-270, 3: 270-360 градусов
    
 }; // class NodeDatabase
 
 #endif // NODE_DATABASE_H
```

---

## File: .\src\PacketManager.cpp
```cpp
/**
 * File: PacketManager.cpp
 * Version: 1.46
 * Изменение: Использование частичных обновлений ble.sendNodeCoords и ble.sendNodeInfo при обработке LoRa пакетов.
 */

 #include "PacketManager.h"
 #include "logger.h"

 PacketManager::PacketManager(NodeDatabase& nodeDB, GpsManager& gps, GeoPacker& packer, BleManager& ble)
     : _nodeDB(nodeDB), _gps(gps), _packer(packer), _ble(ble), _lastTargetId(0) {}

 void PacketManager::processPacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen) {
     switch (header.getType()) {
         case MSG_COORDS:
             handleCoordsPacket(header.senderId, payload);
             _lastTargetId = header.senderId;
             break;
         case MSG_NODE_INFO:
             handleNodeInfoPacket(header.senderId, payload, payloadLen);
             _lastTargetId = header.senderId;
             break;
         case MSG_LEAVE:
             handleLeavePacket(header.senderId, payload);
             break;
         default:
             LOG_WARN("PKT", "Unknown Radio packet type: %d", header.getType());
             break;
     }
 }

 void PacketManager::handleCoordsPacket(uint8_t senderId, const uint8_t* payload) {
     uint32_t packedCoords;
     memcpy(&packedCoords, payload, 4);
     
     // 1. Обновляем локальную базу (отмечаем получение координат)
     _nodeDB.updateNodeCoords(senderId, 0, 0, packedCoords, true);
     
     // 2. ИЗМЕНЕНИЕ 1.46: Распаковываем и шлем частичный апдейт координат по BLE
     const NodeRecord* node = _nodeDB.getNode(senderId);
     if (node != nullptr) {
         float uLat, uLon;
         _packer.unpack(packedCoords, _gps.getLat(), _gps.getLon(), uLat, uLon);
         _ble.sendNodeCoords(senderId, uLat, uLon, node->snr);
     }
 }

 void PacketManager::handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload, size_t payloadLen) {
     if (payloadLen < 2) return;
     uint8_t role = payload[0];
     char name[24];
     size_t nameLen = payloadLen - 1;
     if (nameLen > 23) nameLen = 23;
     memcpy(name, payload + 1, nameLen);
     name[nameLen] = '\0';

     // 1. Обновляем базу
     _nodeDB.updateNodeInfo(senderId, name, role);

     // 2. ИЗМЕНЕНИЕ 1.46: Шлем частичный апдейт инфо по BLE
     _ble.sendNodeInfo(senderId, role, name);
 }

 void PacketManager::handleLeavePacket(uint8_t senderId, const uint8_t* payload) {
     _nodeDB.removeNode(senderId);
     _ble.sendNodeDelete(senderId); 
     LOG_INFO("DISPATCH", "Node %d left. BLE DELETE sent.", senderId);
 }

 uint8_t PacketManager::getLastTargetId() const { return _lastTargetId; }
 void PacketManager::clearLastTargetId() { _lastTargetId = 0; }
 //PacketManager.cpp
```

---

## File: .\src\PacketManager.h
```cpp
/**
 * File: PacketManager.h
 * Version: 1.41
 * Description: Изолированный класс для парсинга и маршрутизации входящих пакетов.
 * Изменение: Добавлена ссылка на BleManager для уведомления Оператора о выходе узлов (v1.41).
 */
 #ifndef PACKET_MANAGER_H
 #define PACKET_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"
 #include "GpsManager.h"
 #include "GeoPacker.h"
 #include "BleManager.h" // ИЗМЕНЕНИЕ 1.41
 
 class PacketManager {
 public:
     // ИЗМЕНЕНИЕ 1.41: Конструктор теперь принимает ссылку на BleManager
     PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer, BleManager& ble);
 
     void processPacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen);
     
     uint8_t getLastTargetId() const;
     void clearLastTargetId();
 
 private:
     NodeDatabase& _nodeDB;
     GpsManager& _gps;
     GeoPacker& _packer;
     BleManager& _ble; // ИЗМЕНЕНИЕ 1.41
     uint8_t _lastTargetId;
 
     void handleCoordsPacket(uint8_t senderId, const uint8_t* payload);
     void handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload, size_t payloadLen);
     void handleLeavePacket(uint8_t senderId, const uint8_t* payload);
 };
 
 #endif // PACKET_MANAGER_H
```

---

## File: .\src\PowerManager.cpp
```cpp
/**
 * File: PowerManager.cpp
 * Version: 1.34 
 * Изменение: Реализована логика измерения заряда для плат с AXP2101 и прямой расчет по АЦП для T-Energy S3.
 * Description: Реализация класса управления питанием.
 */
 #include "PowerManager.h"
 #include "logger.h"
 
 PowerManager::PowerManager() {
     // Конструктор пока пуст, вся работа в init()
 }
 
 bool PowerManager::init() {
 #if HAS_PMU
     // AXP2101 сидит на той же шине I2C, что и дисплей (T-Beam v1.1)
     if (_pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
         LOG_INFO("PWR", "AXP2101 PMU initialized successfully.");
         
         // Включаем питание LoRa (LDO линия ALDO2)
         _pmu.setALDO2Voltage(3300); 
         _pmu.enableALDO2();
         
         // Включаем питание GPS (LDO линия ALDO3)
         _pmu.setALDO3Voltage(3300); 
         _pmu.enableALDO3();
         
         // Отключаем неиспользуемые линии для экономии
         _pmu.disableALDO4(); 
         
         // Включаем АЦП для замера батареи 
         _pmu.enableSystemVoltageMeasure();  
         return true;
     }
     LOG_WARN("PWR", "AXP2101 PMU initialization FAILED!");
     return false;
 #else
     // Если на плате нет контроллера питания (как на T-Energy S3), 
     // просто имитируем успешную инициализацию (Dummy mode).
     LOG_INFO("PWR", "No PMU on this board. Direct power is used.");
     return true; 
 #endif
 }
 
 // Жесткий аппаратный сброс питания GPS-модуля (Используется для вывода Ublox из зависших состояний)
 void PowerManager::cycleGpsPower() {
 #if HAS_PMU
     LOG_INFO("PWR", "Cycling GPS power (ALDO3)...");
     _pmu.disableALDO3(); 
     delay(2000); 
     _pmu.enableALDO3(); 
     delay(2000); 
     LOG_INFO("PWR", "GPS power restored.");
 #else
     // Если PMU нет, мы не можем аппаратно передернуть питание GPS.
     // Оставляем только логирование. Встроенный программный Rescue Mode (через UBX_FACTORY_RESET) в GpsManager отработает сам.
     LOG_WARN("PWR", "Cannot cycle GPS power: No PMU hardware.");
 #endif
 }
 
 // Получение процента заряда батареи
 uint8_t PowerManager::getBatteryPercent() {
 #if HAS_PMU
     return _pmu.getBatteryPercent();
 #else
     // Для плат без PMU (T-Energy S3) вычисляем процент по напряжению.
     // Полностью заряженный Li-Ion ~ 4.2V (4200 mV), полностью разряженный ~ 3.3V (3300 mV).
     uint16_t mv = getBatteryVoltage();
     if (mv >= 4200) return 100;
     if (mv <= 3300) return 0;
     return (uint8_t)map(mv, 3300, 4200, 0, 100);
 #endif
 }
 
 // Получение напряжения батареи в милливольтах
 uint16_t PowerManager::getBatteryVoltage() {
 #if HAS_PMU
     return _pmu.getBattVoltage(); // Возвращает милливольты через драйвер
 #else
     // Для плат T-Energy S3: аппаратный делитель напряжения 1/2 подключен к GPIO 3.
     // analogReadMilliVolts возвращает напряжение на самом пине. Умножаем на 2.
     uint32_t adc_mv = analogReadMilliVolts(3);
     return (uint16_t)(adc_mv * 2);
 #endif
 }
 //PowerManager.cpp
```

---

## File: .\src\PowerManager.h
```cpp
/**
 * File: PowerManager.h
 * Version: 1.34 
 * Изменение: Добавлены методы чтения напряжения и процента заряда батареи для отправки телеметрии.
 * Description: Изолированный класс для управления питанием.
 */
 #ifndef POWER_MANAGER_H
 #define POWER_MANAGER_H
 
 #include <Arduino.h>
 #include <Wire.h>
 #include "configuration.h"
 
 // Подключаем библиотеку PMU только если чип аппаратно присутствует (T-Beam)
 #if HAS_PMU
 #include <XPowersLib.h>
 #endif
 
 class PowerManager {
 public:
     PowerManager();
 
     // Инициализация чипа и включение нужных линий питания
     bool init();
 
     // Сброс питания на линии ALDO3 (используется для спасения зависшего GPS)
     void cycleGpsPower();
 
     // Получение телеметрии батареи (Реализовано в v1.34)
     uint8_t getBatteryPercent();
     uint16_t getBatteryVoltage();
 
 private:
 #if HAS_PMU
     XPowersAXP2101 _pmu; // Экземпляр драйвера питания
 #endif
 };
 
 #endif // POWER_MANAGER_H
```

---

## File: .\src\RadioManager.cpp
```cpp
/**
 * File: RadioManager.cpp
 * Version: 1.31 Изменение: Внедрение ручного управления антенным коммутатором (Manual RF Switch).
 * Description: Реализация класса управления LoRa.
 */
 #include "RadioManager.h"

 // Конструктор: динамически выделяем память под Module и передаем его в объект радио.
 // Используется универсальный LORA_IRQ, который в config.h настроен на нужный пин для каждой платы.
 RadioManager::RadioManager() : 
     _mod(new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY)), 
     _radio(_mod) {
 }
 
 bool RadioManager::init(void (*isr)(void)) {
     // Инициализация LoRa на частоте 433 MHz
     int state = _radio.begin(433.0);
     if (state == RADIOLIB_ERR_NONE) {
 
 #ifdef LORA_CHIP_SX1268
         // Единые (агрессивные) настройки SX1268 (E22) для обеих аппаратных платформ (максимальная дальность)
         _radio.setDio2AsRfSwitch(false);    
         _radio.setBandwidth(125.0);         
         _radio.setSpreadingFactor(9);       
         _radio.setCodingRate(5);            
         _radio.setSyncWord(0x2B);           
         _radio.setPreambleLength(16);       
         _radio.setOutputPower(22);          // Максимальная мощность передачи (22 dBm)
         _radio.setCurrentLimit(140);        
         _radio.setTCXO(1.8);
         
         // ИЗМЕНЕНИЕ 1.31: Отключаем автоматическое управление пинами в RadioLib, 
         // так как на быстрых чипах ESP32-S3 это приводит к рассинхронизации
         // _radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         
         // Настраиваем пины управления антенным коммутатором вручную как OUTPUT
         pinMode(LORA_RXEN, OUTPUT);
         pinMode(LORA_TXEN, OUTPUT);

         // Устанавливаем коммутатор в режим приема (LNA открыт, приемный усилитель включен) по умолчанию
         digitalWrite(LORA_RXEN, HIGH);
         digitalWrite(LORA_TXEN, LOW);
         
         _radio.setRxBoostedGainMode(true);
 #endif
 
         // Подключаем аппаратное прерывание (вызов коллбэка при получении пакета) и стартуем
         _radio.setPacketReceivedAction(isr); 
         _radio.startReceive();
         return true;
     }
     return false;
 }
 
 void RadioManager::startReceive() { 
 #ifdef LORA_CHIP_SX1268
     // Гарантируем, что коммутатор в режиме приема (RX) при ручном перезапуске прослушивания
     digitalWrite(LORA_TXEN, LOW);
     digitalWrite(LORA_RXEN, HIGH);
 #endif
     _radio.startReceive(); 
 }
 
 // Перевод радио в спящий/ждущий режим
 void RadioManager::standby() { _radio.standby(); }
 size_t RadioManager::getPacketLength() { return _radio.getPacketLength(); }
 int RadioManager::readData(uint8_t* buffer, size_t len) { return _radio.readData(buffer, len); }
 
 // Метод передачи пакета
 int RadioManager::transmit(uint8_t* buffer, size_t len) { 
 #ifdef LORA_CHIP_SX1268
     // ИЗМЕНЕНИЕ 1.31: Мгновенно переключаем антенный коммутатор на передачу (TX).
     // Открываем передающий усилитель (PA) и глушим приемный (LNA).
     digitalWrite(LORA_RXEN, LOW);
     digitalWrite(LORA_TXEN, HIGH);
 #endif

     // Блокирующий вызов отправки через RadioLib
     int state = _radio.transmit(buffer, len); 

 #ifdef LORA_CHIP_SX1268
     // ИЗМЕНЕНИЕ 1.31: Сразу после завершения отправки возвращаем коммутатор в режим приема (RX),
     // чтобы не пропустить входящие пакеты
     digitalWrite(LORA_TXEN, LOW);
     digitalWrite(LORA_RXEN, HIGH);
 #endif

     return state;
 }
 
 float RadioManager::getSNR() { return _radio.getSNR(); }

 //RadioManager.cpp
```

---

## File: .\src\RadioManager.h
```cpp
/**
 * File: RadioManager.h
 * Version: 1.31 Изменение: Подготовка к ручному управлению антенным коммутатором.
 * Description: Изолированный класс (обертка) для управления радиомодулем LoRa.
 */
 #ifndef RADIO_MANAGER_H
 #define RADIO_MANAGER_H
 
 #include <Arduino.h>
 #include <RadioLib.h>
 #include "configuration.h"
 
 class RadioManager {
 public:
     RadioManager();
 
     // Инициализация. Принимает функцию прерывания (коллбэк ISR). Возвращает true при успехе.
     bool init(void (*isr)(void));
 
     // Проброс базовых методов RadioLib наружу для использования в main.cpp
     void startReceive();
     void standby();
     size_t getPacketLength();
     int readData(uint8_t* buffer, size_t len);
     int transmit(uint8_t* buffer, size_t len);
     float getSNR();
 
 private:
     Module* _mod;
 
     // Инициализация класса из RadioLib на основе конфигурации платы
 #ifdef LORA_CHIP_SX1268
     SX1268 _radio;
 #elif defined(LORA_CHIP_SX1278)
     SX1278 _radio; // Оставлено для будущей совместимости, если потребуется чип E32
 #endif
 
 };
 
 #endif // RADIO_MANAGER_H
```

---

## File: .\src\Retranslation.cpp
```cpp
/**
 * File: Retranslation.cpp
 * Version: 1.37 
 * Изменение: Валидация пакетов теперь учитывает переменный размер Payload (min/max).
 * Description: Реализация класса фильтрации эфира ("Таможня эфира").
 */
 #include "Retranslation.h"
 #include "NodeDatabase.h" 
 #include "logger.h"
 #include "configuration.h"
 
 Retranslation::Retranslation() {
     // Инициализация кольцевого буфера анти-дубликатора
     head = 0;
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         history[i].senderId = 0;
         history[i].msgSeq = 0;
     }
 } 
  
 // Анти-дубликатор: Проверяет, был ли этот пакет (ID + номер) обработан ранее.
 bool Retranslation::isDuplicate(uint8_t senderId, uint8_t msgSeq) {
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         if (history[i].senderId == senderId && history[i].msgSeq == msgSeq) return true; 
     }
     
     // Если пакет новый, записываем его в кольцевой буфер
     history[head].senderId = senderId;
     history[head].msgSeq = msgSeq;
     head++;
     if (head >= HISTORY_SIZE) head = 0;
     return false; 
 } 
  
 // Базовая валидация пакета по размеру полезной нагрузки
 bool Retranslation::isValidPacket(uint8_t msgType, size_t payloadLen) const {
     if (msgType != MSG_NODE_INFO && msgType != MSG_COORDS && msgType != MSG_LEAVE) return false;
     MessagePolicy policy = getMessagePolicy(msgType);
     
     // ИЗМЕНЕНИЕ 1.37: Проверка попадания длины полезной нагрузки в разрешенный диапазон
     return (payloadLen >= policy.minSize && payloadLen <= policy.maxSize);
 } 
  
 // Главный метод: Решение о том, должен ли локальный узел ретранслировать пакет
 bool Retranslation::shouldRetransmit(const NavigaHeader& header, const NodeDatabase& nodeDB, uint8_t myNodeType, float mySpeed) const {
     // Проверка: маршрутизируется ли в принципе данный тип сообщений
     MessagePolicy policy = getMessagePolicy(header.getType());
     if (!policy.isRoutable) return false;
 
     // Проверка Time-to-Live (TTL): Если жизни не осталось, пакет сбрасывается
     if (header.getTTL() <= 1) {
         LOG_INFO("RELAY", "Packet Seq %d dropped: TTL expired.", header.msgSeq);
         return false;
     } 
     
     // --- фильтр количества узлов ---
     // Если в сети мало узлов (меньше 3), ретрансляция не имеет смысла (все слышат друг друга напрямую)
     if (nodeDB.getActiveNodesCount() < 3) {
         LOG_INFO("RELAY", "Packet Seq %d dropped: Network too small (%d nodes).", header.msgSeq, nodeDB.getActiveNodesCount());
         return false;
     }
 
     // --- РОЛЕВЫЕ ФИЛЬТРЫ ТРЕКЕРА (Шаг 2) ---
     if (myNodeType == NODE_TRACKER) {
         // Трекер бежит: полностью отключаем функцию радиоудлинителя для экономии батареи и CPU
         if (mySpeed > TRACKER_FAST_SPEED_KMPH) {
             LOG_INFO("RELAY", "Packet Seq %d dropped: We are running Tracker (%.1f km/h).", header.msgSeq, mySpeed);
             return false;
         }
         
         // Трекер идет/стоит: ретранслирует выборочно, отбрасывая "тяжелый" админ-трафик (NodeInfo)
         if (header.getType() == MSG_NODE_INFO) {
             LOG_INFO("RELAY", "Packet Seq %d dropped: Trackers do not relay NODE_INFO.", header.msgSeq);
             return false;
         }
         
         // Запрещаем Трекеру удлинять стационарную магистраль
         // Проверяем именно того, от кого ФИЗИЧЕСКИ пришел пакет (relayId)
         const NodeRecord* relayer = nodeDB.getNode(header.relayId);
         if (relayer != nullptr && relayer->type == NODE_RELAY) {
             LOG_INFO("RELAY", "Packet Seq %d dropped: Trackers do not relay for RELAY nodes.", header.msgSeq);
             return false;
         }
     }
 
     // Фильтр Компактной группы: Если максимальное расстояние в группе меньше 200м, ретрансляция не нужна
     if (nodeDB.getCachedMaxDist() < MAX_DIRECT_CONNECT_METERS) {
         LOG_INFO("RELAY", "Packet Seq %d dropped: Group is dense (Max %.1fm).", header.msgSeq, nodeDB.getCachedMaxDist());
         return false;
     }
 
     // Векторный фильтр (Строгий Географический тупик)
     // Проверяет, есть ли узлы "за спиной" относительно источника пакета
     // Отключение векторного фильтра для роли NODE_RELAY (Магистраль светит во все стороны)
     if (myNodeType != NODE_RELAY) {
         if (!nodeDB.hasNodesInOppositeDirection(header.relayId)) {
             LOG_INFO("RELAY", "Packet Seq %d dropped: Geographic dead end (No nodes opposite to %d).", header.msgSeq, header.relayId);
             return false;
         }
     }
     
     // Если пакет прошел все фильтры, возвращаем true
     return true; 
 } //Retranslation.cpp
```

---

## File: .\src\Retranslation.h
```cpp
/**
 * File: Retranslation.h
 * Version: 1.18 Изменение: Добавлены параметры роли и скорости в shouldRetransmit.
 * Description: Заголовочный файл класса фильтрации эфира.
 * Содержит ТОЛЬКО логику валидации и анти-дубликатор. 
 * Вся работа с очередями перенесена в TxManager.
 */
 #ifndef RETRANSLATION_H
 #define RETRANSLATION_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 
 // --- НАСТРОЙКИ АНТИ-ДУБЛИКАТОРА ---
 const uint16_t HISTORY_SIZE = 300; // Хранит последние 300 уникальных пакетов
 
 // Структура записи в кольцевом буфере анти-дубликатора
 struct PacketRecord {
     uint8_t senderId; 
     uint8_t msgSeq;   
 };

 // НОВОЕ: Forward Declaration (упреждающее объявление) класса базы узлов
class NodeDatabase;

 class Retranslation {
 public:
     Retranslation();
 
     // Методы валидации (Таможня)
     bool isDuplicate(uint8_t senderId, uint8_t msgSeq);
     bool isValidPacket(uint8_t msgType, size_t payloadLen) const;
     
    // Изменение 1.18: Передаем свою роль и скорость для принятия ролевых решений
    bool shouldRetransmit(const NavigaHeader& header, const NodeDatabase& nodeDB, uint8_t myNodeType, float mySpeed) const;

 private:
     PacketRecord history[HISTORY_SIZE]; // Кольцевой буфер истории
     uint16_t head;                      // Указатель на голову кольцевого буфера
 }; 
 
 #endif // RETRANSLATION_H
```

---

## File: .\src\TxManager.cpp
```cpp
/**
 * File: TxManager.cpp
 * Version: 1.38 
 * Изменение: Умная упаковка пакетов NODE_INFO переменной длины (до 24 байт) (Шаг 2).
 * Description: Реализация TxManager (MAC-диспетчер и управление очередями отправки).
 */
 #include "TxManager.h"
 #include "logger.h"
 
 extern volatile bool receivedFlag; // Флаг прерывания из main.cpp для определения занятости эфира
 
 TxManager::TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq)
     : _radio(radio), _packer(packer), _myNodeId(nodeId), _myMsgSeq(msgSeq) {
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         _queue[i].isActive = false;
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
     _activeJobIndex = -1;
     _jitterStartTime = 0;
     _jitterDelay = 0;
 } // TxManager::TxManager()
 
 // Базовый метод постановки задачи в очередь (CSMA/CA)
 bool TxManager::enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs) {
     if (payloadLen > MAX_PAYLOAD_SIZE) return false;
 
     // Ищем свободный слот в очереди
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (!_queue[i].isActive) {
             _queue[i].header = header;
             if (payloadLen > 0 && payload != nullptr) {
                 memcpy(_queue[i].payload, payload, payloadLen);
             } // if (payloadLen > 0 && ...)
             _queue[i].payloadLen = payloadLen;
             _queue[i].priority = priority;
             _queue[i].readyTime = millis() + delayMs; // Время, когда пакет можно начать обрабатывать
             _queue[i].isActive = true;
             return true;
         } // if (!_queue[i].isActive)
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
     
     LOG_WARN("TX", "TxQueue is FULL! Dropping packet.");
     return false;
 } // TxManager::enqueue()
 
 // Упаковка и постановка в очередь информации об узле (Heartbeat / Collision reset)
 void TxManager::sendNodeInfo(const char* nodeName, uint8_t nodeType, TxPriority priority) {
     NavigaHeader txHeader;
     txHeader.senderId = _myNodeId;
     txHeader.relayId = _myNodeId; // Изначально мы являемся и отправителем и ретранслятором
     txHeader.msgSeq = _myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_NODE_INFO, DEFAULT_TTL);
 
     // ИЗМЕНЕНИЕ 1.38: Формируем пакет переменной длины
     uint8_t payloadBuf[MAX_PAYLOAD_SIZE];
     payloadBuf[0] = nodeType;
     
     size_t nameLen = strlen(nodeName);
     if (nameLen > MAX_PAYLOAD_SIZE - 2) {
         nameLen = MAX_PAYLOAD_SIZE - 2; // Оставляем место под байт роли и нуль-терминатор
     }
     memcpy(&payloadBuf[1], nodeName, nameLen);
     payloadBuf[1 + nameLen] = '\0'; // Гарантированный нуль-терминатор
     
     size_t actualLen = 1 + nameLen + 1;
 
     enqueue(txHeader, payloadBuf, actualLen, priority, 0);
     LOG_INFO("TX", "Enqueued NODE_INFO: Name=%s, Type=%d, Len=%d", nodeName, nodeType, actualLen);
 } // TxManager::sendNodeInfo()
 
 // Упаковка и постановка в очередь координат
 void TxManager::sendCoords(float lat, float lon, TxPriority priority) {
     NavigaHeader txHeader;
     txHeader.senderId = _myNodeId;
     txHeader.relayId = _myNodeId;
     txHeader.msgSeq = _myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_COORDS, DEFAULT_TTL);
 
     // Сжатие координат перед постановкой
     uint32_t packedCoords = _packer.pack(lat, lon);
     enqueue(txHeader, (const uint8_t*)&packedCoords, sizeof(uint32_t), priority, 0);
 } // TxManager::sendCoords()
 
 // Постановка в очередь чужого пакета (Ретрансляция) с заданным окном задержки (Джиттером)
 bool TxManager::enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, uint32_t delayMs) {
     if (payloadLen > MAX_PAYLOAD_SIZE) return false;
 
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (!_queue[i].isActive) {
             _queue[i].header = header;
             
             // Декрементируем TTL (Time-to-Live)
             uint8_t currentTTL = header.getTTL();
             uint8_t newTTL = (currentTTL > 0) ? (currentTTL - 1) : 0;
             _queue[i].header.setTypeAndTTL(header.getType(), newTTL);
             
             // Записываем СЕБЯ как последнего ретранслятора
             _queue[i].header.relayId = _myNodeId;
 
             if (payloadLen > 0 && payload != nullptr) {
                 memcpy(_queue[i].payload, payload, payloadLen);
             } // if (payloadLen > 0 && payload != nullptr)
             
             _queue[i].payloadLen = payloadLen;
             _queue[i].priority = TX_RELAY; // Приоритет ретрансляции
             
             // Задержка (Adaptive Jitter) теперь сразу прибавляется к времени готовности задачи
             _queue[i].readyTime = millis() + delayMs;
             
             _queue[i].isActive = true;
             return true;
         } // if (!_queue[i].isActive)
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
     return false;
 } // TxManager::enqueueRelay()
 
 // Implicit ACK (Подавление перехватом). Вызывается из main, если в эфире услышан дубликат.
 void TxManager::abortRelay(uint8_t senderId, uint8_t msgSeq) {
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (_queue[i].isActive && _queue[i].header.senderId == senderId && _queue[i].header.msgSeq == msgSeq) {
             _queue[i].isActive = false; // Удаляем задачу из очереди
             if (_activeJobIndex == i) {
                 _activeJobIndex = -1;   // Если она уже была активна на отправку, сбрасываем состояние
             } // if (_activeJobIndex == i)
             LOG_INFO("QUEUE", "Relay ABORTED for Seq %d (Suppressed by network)", msgSeq);
             return;
         } // if (_queue[i].isActive && ...)
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
 } // TxManager::abortRelay()
 
 // Диспетчер отправки. Вызывается регулярно в loop(). Реализует механизм CSMA/CA.
 void TxManager::processQueue() {
     // Защита от коллизий: если радиомодуль ловит шум (принимает пакет), 
     // мы сбрасываем попытку отправки и ждем освобождения эфира.
     if (receivedFlag) {
         _activeJobIndex = -1;
         return;
     } // if (receivedFlag)
 
     uint32_t now = millis();
 
     // Если нет активной задачи в процессе, выбираем самую приоритетную
     if (_activeJobIndex == -1) {
         int8_t bestIndex = -1;
         TxPriority bestPriority = TX_RELAY; 
 
         // Сканируем очередь на наличие готовых к отправке пакетов (с учетом задержки readyTime)
         for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
             if (_queue[i].isActive && now >= _queue[i].readyTime) {
                 if (bestIndex == -1 || _queue[i].priority < bestPriority) {
                     bestIndex = i;
                     bestPriority = _queue[i].priority;
                 } // if (bestIndex == -1 || ...)
             } // if (_queue[i].isActive && now >= _queue[i].readyTime)
         } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
 
         // Нашли лучшую задачу. Присваиваем ей финальный системный джиттер перед отправкой.
         if (bestIndex != -1) {
             _activeJobIndex = bestIndex;
             _jitterStartTime = now;
             
             // Добавляем микро-задержки для предотвращения одновременного выхода в эфир 
             // нескольких устройств после окончания приема чужого пакета
             switch(bestPriority) {
                 case TX_CRITICAL: _jitterDelay = random(10, 50); break;
                 case TX_HIGH:     _jitterDelay = random(50, 150); break;
                 case TX_NORMAL:   _jitterDelay = random(100, 300); break;
                 case TX_RELAY: {
                     // Джиттер уже был учтен в readyTime при помещении в очередь.
                     // Ставим 0, чтобы передача сработала немедленно при выборе задачи.
                     _jitterDelay = 0; 
                     break;
                 }
             } // switch(bestPriority)
         } // if (bestIndex != -1)
     } // if (_activeJobIndex == -1)
 
     // Если активная задача есть и таймер джиттера истек — выходим в эфир
     if (_activeJobIndex != -1) {
         if (now - _jitterStartTime >= _jitterDelay) {
             
             // Подготавливаем буфер для физической передачи
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
 
             _radio.standby(); // Переводим радио в спящий режим перед передачей
             _radio.transmit(txBuffer, totalLen); // Блокирующая передача!
             receivedFlag = false; // Очищаем флаг приема, который мог подняться из-за наводок при передаче
             _radio.startReceive(); // Возвращаем в режим приема
 
             // Задача выполнена, освобождаем слот
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
 * Version: 1.38 
 * Изменение: MAX_PAYLOAD_SIZE увеличен до 24 байт для поддержки кириллицы (Шаг 2).
 * Description: Единый конвейер (Очередь CSMA/CA) для отправки пакетов с поддержкой типа узла.
 */
 #ifndef TX_MANAGER_H
 #define TX_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "RadioManager.h"
 #include "GeoPacker.h"
 
 // Приоритеты для очереди передачи
 enum TxPriority {
     TX_CRITICAL = 0, // Немедленная смена ID при коллизии
     TX_HIGH = 1,     // Отправка собственных координат в движении
     TX_NORMAL = 2,   // Сервисные сообщения (Heartbeat NodeInfo)
     TX_RELAY = 3     // Ретрансляция чужих сообщений (Самый низкий приоритет)
 }; // enum TxPriority
 
 const uint8_t TX_QUEUE_SIZE = 15;    // Максимальный размер очереди
 const uint8_t MAX_PAYLOAD_SIZE = 24; // ИЗМЕНЕНИЕ 1.38: Максимальный размер полезной нагрузки (Payload)
 
 // Структура одной задачи на отправку
 struct TxJob {
     bool isActive;
     TxPriority priority;
     uint32_t readyTime; // Время (millis()), когда задача готова к отправке (учитывает джиттер)
     NavigaHeader header;
     uint8_t payload[MAX_PAYLOAD_SIZE];
     size_t payloadLen;
 }; // struct TxJob 
 
 class TxManager {
 public:
     TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq);
 
     // Методы добавления в очередь разных типов сообщений
     void sendNodeInfo(const char* nodeName, uint8_t nodeType, TxPriority priority = TX_NORMAL);
     void sendCoords(float lat, float lon, TxPriority priority = TX_HIGH);
     
     bool enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, uint32_t delayMs);
     
     // Подавление перехватом (Удаление ретрансляции, если в эфире пойман дубль)
     void abortRelay(uint8_t senderId, uint8_t msgSeq);
     
     // Главный диспетчер очередей
     void processQueue();
 
 private:
     RadioManager& _radio;
     GeoPacker& _packer;
     uint8_t& _myNodeId; // Ссылка на глобальный ID устройства
     uint8_t& _myMsgSeq; // Ссылка на глобальный счетчик пакетов
 
     TxJob _queue[TX_QUEUE_SIZE];
     
     int8_t _activeJobIndex;
     uint32_t _jitterStartTime;
     uint32_t _jitterDelay;
 
     // Внутренний метод постановки в очередь
     bool enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs = 0);
 }; // class TxManager
 
 #endif // TX_MANAGER_H
```

---

## File: .\src\BleConfig.h
```cpp
/**
 * File: BleConfig.h
 * Version: 1.26
 * Description: UUID сервиса и характеристик для связи со смартфоном по Bluetooth (UC-02).
 */
 #ifndef BLE_CONFIG_H
 #define BLE_CONFIG_H
 
 // UUID Сервиса, по которому мобильное приложение (Оператор) находит Донгл
 #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
 
 // UUID Характеристики RX (Прием от телефона). Свойства: WRITE
 #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
 
 // UUID Характеристики TX (Отправка в телефон). Свойства: NOTIFY
 #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
  
 // Базовое имя, которое будет расширяться MAC-суффиксом в BleManager
 #define BLE_DEVICE_NAME        "Naviga-Dongle"
 
 #endif
```

---

## File: .\src\BleProtocol.h
```cpp
/**
 * File: BleProtocol.h
 * Version: 1.46.3
 * Изменение: Единый заголовочный файл протокола со статусами и структурами.
 * Description: Определение протокола связи.
 */

 #ifndef BLE_PROTOCOL_H
 #define BLE_PROTOCOL_H
 
 #include <Arduino.h>
 
 // Состояния BLE (единое определение)
 enum BleStatus {
     BLE_OFF,
     BLE_UNPAIRED,
     BLE_CONNECTED
 };
 
 #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
 #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
 #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
 
 enum BleOpCode {
     CMD_SET_IDENTITY    = 0x01,
     CMD_SET_SYS_CONFIG  = 0x02,
     CMD_ACTION_RESET    = 0x03,
     CMD_ACTION_CLEAR_DB = 0x04,
     CMD_REQ_FULL_SYNC   = 0x05,
     CMD_REQ_IDENTITY    = 0x06,
     CMD_REQ_SYS_CONFIG  = 0x07,
 
     EVT_MY_STATUS       = 0x10,
     EVT_NODE_UPDATE     = 0x11, 
     EVT_IDENTITY        = 0x12,
     EVT_SYS_CONFIG      = 0x13,
     EVT_NODE_DELETE     = 0x14,
     EVT_NODE_COORDS     = 0x15, 
     EVT_NODE_INFO       = 0x16  
 };
 
 #pragma pack(push, 1)
 
 struct BleIdentity {
     uint8_t opCode;
     uint8_t myNodeId;
     char myName[24];
     uint8_t myRole;
 };
 
 struct BleSysConfig {
     uint8_t opCode;
     uint32_t txIntervalMoving;
     uint32_t txIntervalStill;
     uint32_t nodeConnectionTimeout;
     uint32_t nodeActiveTimeoutMs;
 };
 
 struct BleEvtMyStatus {
     uint8_t opCode;
     uint8_t gpsValid;
     uint8_t satellites;
     uint8_t batteryPercent;
     uint16_t batteryVoltage;
 };
 
 struct BleEvtNodeUpdate {
     uint8_t opCode;
     uint8_t nodeId;
     uint8_t nodeRole;
     char nodeName[24];
     float lat;
     float lon;
     float snr;
     uint32_t lastSeenAge;
 };
 
 struct BleEvtNodeDelete {
     uint8_t opCode;
     uint8_t nodeId;
 };
 
 struct BleEvtNodeCoords {
     uint8_t opCode;
     uint8_t nodeId;
     float lat;
     float lon;
     float snr;
 };
 
 struct BleEvtNodeInfo {
     uint8_t opCode;
     uint8_t nodeId;
     uint8_t nodeRole;
     char nodeName[24];
 };
 
 #pragma pack(pop)
 
 #endif // BleProtocol.h
```

---

## File: .\src\BleManager.h
```cpp
/**
 * File: BleManager.h
 * Version: 1.46.3
 * Изменение: Прототипы для частичных обновлений. Убран дубликат BleStatus.
 * Description: Заголовочный файл менеджера Bluetooth.
 */

 #ifndef BLE_MANAGER_H
 #define BLE_MANAGER_H
 
 #include <NimBLEDevice.h>
 #include "BleProtocol.h" 
 
 class BleManager {
 public:
     BleManager();
     void init();
     void process();
     
     BleStatus getBleStatus();
     
     void sendIdentity(uint8_t nodeId, const char* name, uint8_t role);
     void sendSysConfig(uint32_t txMoving, uint32_t txStill, uint32_t connTimeout, uint32_t activeTimeout);
     void sendNodeUpdate(const BleEvtNodeUpdate& nodeData);
     void sendNodeDelete(uint8_t nodeId);
     void sendMyStatus(uint8_t gpsValid, uint8_t satellites, uint8_t batteryPercent, uint16_t batteryVoltage);
     
     void sendNodeCoords(uint8_t id, float lat, float lon, float snr);
     void sendNodeInfo(uint8_t id, uint8_t role, const char* name);
 
     bool hasNewIdentity;
     BleIdentity newIdentity;
     
     bool hasNewSysConfig;
     BleSysConfig newSysConfig;
     
     bool requestFullSync;
     bool requestReset;
     bool requestClearDB;
     bool requestIdentitySync;
     bool requestSysConfigSync;
 
     char macSuffix[5];
 
 private:
     NimBLEServer* pServer;
     NimBLECharacteristic* pTxCharacteristic;
     NimBLECharacteristic* pRxCharacteristic;
     bool _isConnected;
 
     class ServerCallbacks;
     class RxCallbacks;
 };
 
 #endif // BleManager.h
```

---

## File: .\src\BleManager.cpp
```cpp
/**
 * File: BleManager.cpp
 * Version: 1.46
 * Изменение: Реализованы методы sendNodeCoords и sendNodeInfo для частичных обновлений.
 */

 #include "BleManager.h"
 #include "logger.h" 
 #include <esp_mac.h> 

 class BleManager::ServerCallbacks : public NimBLEServerCallbacks {
     BleManager* _manager;
 public:
     ServerCallbacks(BleManager* manager) : _manager(manager) {}
     
     void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
         _manager->_isConnected = true;
         LOG_INFO("BLE", "Smartphone connected!");
     }

     void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
         _manager->_isConnected = false;
         LOG_INFO("BLE", "Smartphone disconnected. Restarting advertising...");
         NimBLEDevice::startAdvertising(); 
     }
 };

 class BleManager::RxCallbacks : public NimBLECharacteristicCallbacks {
     BleManager* _manager;
 public:
     RxCallbacks(BleManager* manager) : _manager(manager) {}
 
     void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
         std::string rxValue = pCharacteristic->getValue();
         if (rxValue.length() > 0) {
             uint8_t opCode = rxValue[0];
             switch (opCode) {
                 case CMD_SET_IDENTITY:
                     if (rxValue.length() == sizeof(BleIdentity)) {
                         memcpy(&_manager->newIdentity, rxValue.data(), sizeof(BleIdentity));
                         _manager->hasNewIdentity = true; 
                     }
                     break;
                 case CMD_SET_SYS_CONFIG:
                     if (rxValue.length() == sizeof(BleSysConfig)) {
                         memcpy(&_manager->newSysConfig, rxValue.data(), sizeof(BleSysConfig));
                         _manager->hasNewSysConfig = true; 
                     }
                     break;
                 case CMD_REQ_FULL_SYNC: _manager->requestFullSync = true; break;
                 case CMD_REQ_IDENTITY:  _manager->requestIdentitySync = true; break;
                 case CMD_REQ_SYS_CONFIG: _manager->requestSysConfigSync = true; break;
                 case CMD_ACTION_RESET:   _manager->requestReset = true; break;
                 case CMD_ACTION_CLEAR_DB: _manager->requestClearDB = true; break;
             }
         }
     }
 };
 
 BleManager::BleManager() : 
     pServer(nullptr), pTxCharacteristic(nullptr), pRxCharacteristic(nullptr),
     _isConnected(false), hasNewIdentity(false), hasNewSysConfig(false),
     requestFullSync(false), requestReset(false), requestClearDB(false),
     requestIdentitySync(false), requestSysConfigSync(false) {
         macSuffix[0] = '\0';
     }
 
 void BleManager::init() {
     uint8_t mac[6];
     esp_read_mac(mac, ESP_MAC_BT);
     snprintf(macSuffix, sizeof(macSuffix), "%02X%02X", mac[4], mac[5]);
     char devName[20];
     snprintf(devName, sizeof(devName), "Naviga-%s", macSuffix);
     NimBLEDevice::init(devName);
     pServer = NimBLEDevice::createServer();
     pServer->setCallbacks(new ServerCallbacks(this));
     NimBLEService* pService = pServer->createService(SERVICE_UUID);
     pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
     pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
     pRxCharacteristic->setCallbacks(new RxCallbacks(this));
     pServer->start(); 
     NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
     pAdvertising->setName(devName);
     pAdvertising->addServiceUUID(SERVICE_UUID);
     pAdvertising->start();
     LOG_INFO("SYS", "BLE Initialized. Name: %s", devName);
 }
      
 BleStatus BleManager::getBleStatus() {
     return _isConnected ? BLE_CONNECTED : BLE_UNPAIRED;
 }
 
 void BleManager::process() {}
 
 void BleManager::sendIdentity(uint8_t nodeId, const char* name, uint8_t role) {
     if (!_isConnected) return;
     BleIdentity packet;
     packet.opCode = EVT_IDENTITY;
     packet.myNodeId = nodeId;
     packet.myRole = role;
     strncpy(packet.myName, name, 23);
     packet.myName[23] = '\0'; 
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleIdentity));
     pTxCharacteristic->notify();
 }
 
 void BleManager::sendSysConfig(uint32_t txMoving, uint32_t txStill, uint32_t connTimeout, uint32_t activeTimeout) {
     if (!_isConnected) return;
     BleSysConfig packet;
     packet.opCode = EVT_SYS_CONFIG;
     packet.txIntervalMoving = txMoving;
     packet.txIntervalStill = txStill;
     packet.nodeConnectionTimeout = connTimeout;
     packet.nodeActiveTimeoutMs = activeTimeout;
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleSysConfig));
     pTxCharacteristic->notify();
 }
 
 void BleManager::sendNodeUpdate(const BleEvtNodeUpdate& nodeData) {
     if (!_isConnected) return;
     pTxCharacteristic->setValue((uint8_t*)&nodeData, sizeof(BleEvtNodeUpdate));
     pTxCharacteristic->notify();
 } 

 // ИЗМЕНЕНИЕ 1.46: Частичное обновление координат (0x15)
 void BleManager::sendNodeCoords(uint8_t id, float lat, float lon, float snr) {
     if (!_isConnected) return;
     BleEvtNodeCoords packet;
     packet.opCode = EVT_NODE_COORDS;
     packet.nodeId = id;
     packet.lat = lat;
     packet.lon = lon;
     packet.snr = snr;
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtNodeCoords));
     pTxCharacteristic->notify();
 }

 // ИЗМЕНЕНИЕ 1.46: Частичное обновление информации об узле (0x16)
 void BleManager::sendNodeInfo(uint8_t id, uint8_t role, const char* name) {
     if (!_isConnected) return;
     BleEvtNodeInfo packet;
     packet.opCode = EVT_NODE_INFO;
     packet.nodeId = id;
     packet.nodeRole = role;
     strncpy(packet.nodeName, name, 23);
     packet.nodeName[23] = '\0';
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtNodeInfo));
     pTxCharacteristic->notify();
 }

 void BleManager::sendNodeDelete(uint8_t nodeId) {
     if (!_isConnected) return;
     BleEvtNodeDelete packet;
     packet.opCode = EVT_NODE_DELETE;
     packet.nodeId = nodeId;
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtNodeDelete));
     pTxCharacteristic->notify();
 }

 void BleManager::sendMyStatus(uint8_t gpsValid, uint8_t satellites, uint8_t batteryPercent, uint16_t batteryVoltage) {
     if (!_isConnected) return;
     BleEvtMyStatus packet;
     packet.opCode = EVT_MY_STATUS;
     packet.gpsValid = gpsValid;
     packet.satellites = satellites;
     packet.batteryPercent = batteryPercent;
     packet.batteryVoltage = batteryVoltage;
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtMyStatus));
     pTxCharacteristic->notify();
 } //BleManager.cpp
```

---

## File: .\src\SettingsManager.h
```cpp
/**
 * File: SettingsManager.h
 * Version: 1.38
 * Description: Управление энергонезависимой памятью (NVS) ESP32.
 * Изменение: Буфер name в SavedNodeRecord расширен до 24 байт (Шаг 2).
 */

 #ifndef SETTINGS_MANAGER_H
 #define SETTINGS_MANAGER_H
 
 #include <Arduino.h>
 #include <Preferences.h>
 #include "configuration.h"
 
 // Форвард-декларация, чтобы не было циклического включения и конфликтов заголовочных файлов
 class NodeDatabase; 
 
 // Структура "снимка" одного узла для сохранения в энергонезависимую память (NVS).
 // Содержит только самую важную информацию (ID, Роль, Имя, Сжатые координаты).
 #pragma pack(push, 1) // Отключение выравнивания для точного размера байтов во Flash-памяти
 struct SavedNodeRecord {
     uint8_t id;
     uint8_t role;
     char name[24]; // ИЗМЕНЕНИЕ 1.38: Увеличен до 24 байт
     uint32_t packedCoords;
 };
 #pragma pack(pop)
 
 class SettingsManager {
 public:
     SettingsManager();
     
     void init(); // Инициализация NVS и загрузка
     void save(); // Сохранение структуры settings
     void factoryReset(); // Полная очистка
 
     // Запись и чтение слепка базы соседей (Roster + Coords)
     // Позволяет сохранить сеть при выключении (Graceful Shutdown)
     void saveNodesSnapshot(const NodeDatabase& db);
     void loadNodesSnapshot(NodeDatabase& db);
 
     // Открытая структура настроек (Загружается в память при старте)
     NavigaSettings settings;
 
 private:
     Preferences _preferences; // Объект библиотеки ESP32 Preferences (обертка над NVS)
     const char* PREFS_NAMESPACE = "naviga";     // Общее пространство имен
     const char* PREFS_CFG_KEY = "sys_config";   // Ключ для системных настроек
     const char* PREFS_NODES_KEY = "nodes_snap"; // Ключ для снимка соседей
 
     void loadDefaults(); // Загрузка дефолтных значений из configuration.h
 };
 
 // Экспорт глобального экземпляра менеджера настроек (Singleton-паттерн для удобного доступа)
 extern SettingsManager settingsManager;
 
 #endif // SETTINGS_MANAGER_H
```

---

## File: .\src\SettingsManager.cpp
```cpp
/**
 * File: SettingsManager.cpp
 * Version: 1.39
 * Изменение: Реализован метод factoryReset() для стирания всех данных и возврата к заводским настройкам (UC-08).
 * Description: Реализация класса управления энергонезависимой памятью (NVS).
 */

 #include "SettingsManager.h"
 #include "NodeDatabase.h"
 #include "logger.h"
 
 // Глобальный экземпляр
 SettingsManager settingsManager;
 
 SettingsManager::SettingsManager() {
     // Конструктор пуст, инициализация в init()
 } // SettingsManager::SettingsManager()
 
 void SettingsManager::init() {
     _preferences.begin(PREFS_NAMESPACE, false); // Открываем NVS в режиме чтения/записи
     
     if (_preferences.isKey(PREFS_CFG_KEY)) {
         // Загружаем настройки, если они уже есть в памяти
         _preferences.getBytes(PREFS_CFG_KEY, &settings, sizeof(NavigaSettings));
         LOG_INFO("SYS", "Settings loaded from NVS. Node ID: %d", settings.nodeId);
     } else {
         // Первый запуск - загружаем дефолты
         LOG_INFO("SYS", "No settings found in NVS. Loading defaults.");
         loadDefaults();
         save(); // Сразу сохраняем структуру, чтобы ключ появился
     } // if (_preferences.isKey(PREFS_CFG_KEY))
     
     _preferences.end();
 } // SettingsManager::init()
 
 void SettingsManager::loadDefaults() {
     settings.nodeId = DEFAULT_NODE_ID;
     settings.nodeType = DEFAULT_NODE_TYPE;
     
     // Безопасное копирование дефолтного имени
     strncpy(settings.nodeName, DEFAULT_NODE_NAME, sizeof(settings.nodeName) - 1);
     settings.nodeName[sizeof(settings.nodeName) - 1] = '\0';
     
     settings.txIntervalMoving = DEFAULT_TX_INTERVAL_MOVING;
     settings.txIntervalStill = DEFAULT_TX_INTERVAL_STILL;
     settings.nodeConnectionTimeout = DEFAULT_NODE_CONN_TIMEOUT;
     settings.nodeActiveTimeoutMs = DEFAULT_NODE_ACTIVE_TIMEOUT;
     
     settings.isConfigured = false;
 } // SettingsManager::loadDefaults()
 
 void SettingsManager::save() {
     _preferences.begin(PREFS_NAMESPACE, false);
     _preferences.putBytes(PREFS_CFG_KEY, &settings, sizeof(NavigaSettings));
     _preferences.end();
 } // SettingsManager::save()
 
 // ИЗМЕНЕНИЕ 1.39: Полная очистка памяти (Factory Reset)
 void SettingsManager::factoryReset() {
     _preferences.begin(PREFS_NAMESPACE, false);
     _preferences.clear(); // Удаляет все ключи (и настройки, и слепки узлов) в пространстве naviga
     _preferences.end();
     
     loadDefaults(); // Сбрасываем структуру в оперативной памяти
     LOG_INFO("SYS", "Factory Reset completed. Flash memory cleared.");
 } // SettingsManager::factoryReset()
 
 void SettingsManager::saveNodesSnapshot(const NodeDatabase& db) {
     SavedNodeRecord snapshot[50]; // Лимит в 50 узлов для слепка
     uint8_t count = 0;
     
     for (int i = 1; i < 255; i++) {
         const NodeRecord* node = db.getNode(i);
         if (node != nullptr && node->isActive) {
             snapshot[count].id = node->nodeId;
             snapshot[count].role = node->type;
             strncpy(snapshot[count].name, node->nodeName, sizeof(snapshot[count].name) - 1);
             snapshot[count].name[sizeof(snapshot[count].name) - 1] = '\0';
             snapshot[count].packedCoords = node->packedCoords;
             
             count++;
             if (count >= 50) break; // Защита от переполнения выделенного массива
         } // if (node != nullptr && node->isActive)
     } // for (int i = 1; i < 255; i++)
     
     if (count > 0) {
         _preferences.begin(PREFS_NAMESPACE, false);
         _preferences.putBytes(PREFS_NODES_KEY, snapshot, count * sizeof(SavedNodeRecord));
         _preferences.end();
         LOG_INFO("SYS", "Saved snapshot of %d nodes to NVS.", count);
     } // if (count > 0)
 } // SettingsManager::saveNodesSnapshot()
 
 void SettingsManager::loadNodesSnapshot(NodeDatabase& db) {
     _preferences.begin(PREFS_NAMESPACE, true); // Открываем только для чтения
     
     if (_preferences.isKey(PREFS_NODES_KEY)) {
         size_t dataLen = _preferences.getBytesLength(PREFS_NODES_KEY);
         uint8_t count = dataLen / sizeof(SavedNodeRecord);
         
         if (count > 0 && count <= 50) {
             SavedNodeRecord snapshot[50];
             _preferences.getBytes(PREFS_NODES_KEY, snapshot, dataLen);
             
             for (uint8_t i = 0; i < count; i++) {
                 db.addNode(snapshot[i].id);
                 db.updateNodeInfo(snapshot[i].id, snapshot[i].name, snapshot[i].role);
                 db.updateNodeCoords(snapshot[i].id, 0.0f, 0.0f, snapshot[i].packedCoords, true);
             } // for (uint8_t i = 0; i < count; i++)
             LOG_INFO("SYS", "Loaded snapshot of %d nodes from NVS.", count);
         } // if (count > 0 && count <= 50)
     } // if (_preferences.isKey(PREFS_NODES_KEY))
     
     _preferences.end();
 } // SettingsManager::loadNodesSnapshot()
```

---

## File: .\src\NetworkManager.h
```cpp
/**
 * File: NetworkManager.h
 * Version: 1.00 (Refactored from main.cpp v1.43)
 * Description: Выделенный менеджер управления сетевой логикой: сканирование, коллизии, качество связи.
 */

 #ifndef NETWORK_MANAGER_H
 #define NETWORK_MANAGER_H
 
 #include <Arduino.h>
 #include "RadioManager.h"
 #include "NodeDatabase.h"
 #include "DisplayManager.h"
 #include "GpsManager.h"
 #include "TxManager.h"
 #include "SettingsManager.h"
 #include "BleManager.h"
 #include "Retranslation.h"
 #include "PacketManager.h"
 
 class NetworkManager {
 public:
     NetworkManager(RadioManager& radio, NodeDatabase& db, DisplayManager& disp, 
                   GpsManager& gps, TxManager& tx, BleManager& ble, Retranslation& router, PacketManager& packetMgr);
 
     // Перенесенные функции управления сетью
     void scanNetwork(bool isWarmStart, uint32_t networkScanDuration, uint8_t& myNodeId);
     void handleCollision(uint8_t& myNodeId, uint8_t& myMsgSeq, uint8_t myNodeType);
     uint32_t calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr);
     int getConnectionQuality(uint8_t targetId, uint8_t myNodeId);
 
 private:
     RadioManager& _radio;
     NodeDatabase& _nodeDB;
     DisplayManager& _display;
     GpsManager& _gps;
     TxManager& _txManager;
     BleManager& _bleManager;
     Retranslation& _router;
     PacketManager& _packetManager;
 };
 
 #endif // NETWORK_MANAGER_H
```

---

## File: .\src\NetworkManager.cpp
```cpp
/**
 * File: NetworkManager.cpp
 * Version: 1.00 (Refactored from main.cpp v1.43)
 * Description: Реализация сетевой логики. Весь код перенесен из main.cpp без изменений алгоритмов.
 */

 #include "NetworkManager.h"
 #include "logger.h"
 
 NetworkManager::NetworkManager(RadioManager& radio, NodeDatabase& db, DisplayManager& disp, 
                                GpsManager& gps, TxManager& tx, BleManager& ble, 
                                Retranslation& router, PacketManager& packetMgr)
     : _radio(radio), _nodeDB(db), _display(disp), _gps(gps), 
       _txManager(tx), _bleManager(ble), _router(router), _packetManager(packetMgr) {
 }
 
 // Расчет джиттера (рандомизированной задержки) для умной ретрансляции пакета
 uint32_t NetworkManager::calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr) {
     uint32_t minMs, maxMs;
     
     // Определяем базовые окна джиттера в зависимости от нашей роли
     if (myRole == NODE_RELAY) {
         minMs = RELAY_JITTER_MIN_MS;
         maxMs = RELAY_JITTER_MAX_MS;
     } else {
         minMs = STALKER_JITTER_MIN_MS;
         maxMs = STALKER_JITTER_MAX_MS;
     }
 
     // VIP-Маршрутизация: Если мы ретранслируем пакет ТРЕКЕРА, даем ему зеленый свет
     if (senderRole == NODE_TRACKER) {
         maxMs /= 2; // Ускоряем в 2 раза
         if (maxMs < minMs) maxMs = minMs; 
     }
 
     // Мапим задержку на основании качества сигнала (SNR)
     // Чем лучше сигнал, тем БОЛЬШЕ задержка (передает дальний узел)
     long snrInt = constrain((long)snr, -15, 5);
     uint32_t baseDelay = map(snrInt, -15, 5, minMs, maxMs);
     
     return baseDelay + random(0, 50); // Добавляем небольшую случайность для разрешения коллизий
 }
 
 // Расчет показателя "качества связи" для вывода на экран (от 1 до 10)
 int NetworkManager::getConnectionQuality(uint8_t targetId, uint8_t myNodeId) {
     if (targetId == myNodeId) return 10; 
 
     const NodeRecord* target = _nodeDB.getNode(targetId);
     if (target == nullptr || targetId == 0) return 0;
     
     // Если от узла давно не было вестей, качество 0
     if (millis() - target->lastSeen > settingsManager.settings.nodeConnectionTimeout) return 0;
     
     if (target->snr <= -99.0f) return 0; 
 
     // Мапим физический SNR в читаемый балл (1-10)
     int q = map((long)target->snr, -11, 5, 1, 10);
     if (q < 1) q = 1;
     if (q > 10) q = 10;
     return q;
 }
 
 // Обработка коллизии: если два узла заняли один ID
 void NetworkManager::handleCollision(uint8_t& myNodeId, uint8_t& myMsgSeq, uint8_t myNodeType) {
     uint8_t oldId = myNodeId;
     _nodeDB.removeNode(oldId); 
     
     // Генерируем новый уникальный ID
     randomSeed(esp_random());
     do {
         myNodeId = random(1, 255);
     } while (_nodeDB.getNode(myNodeId) != nullptr); // Проверяем, свободен ли он в базе
     
     _nodeDB.addNode(myNodeId); 
     
     LOG_WARN("COLLISION", "ID %d is taken! Switched to new ID: %d", oldId, myNodeId);
     
     myMsgSeq = 0; // Сбрасываем счетчик пакетов
     
     char myName[24]; // Буфер расширен до 24 байт
     snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
     
     // Рассылаем новый ID по сети с наивысшим приоритетом
     _txManager.sendNodeInfo(myName, myNodeType, TX_CRITICAL);
     _nodeDB.updateNodeInfo(myNodeId, myName, myNodeType); 
  
     // Сохраняем изменения в энергонезависимую память (NVS)
     settingsManager.settings.nodeId = myNodeId;
     settingsManager.save();
     settingsManager.saveNodesSnapshot(_nodeDB);
 }
 
 // Универсальная функция первоначального сканирования и немого периода (Warm/Cold Start)
 void NetworkManager::scanNetwork(bool isWarmStart, uint32_t networkScanDuration, uint8_t& myNodeId) {
     if (isWarmStart) {
         LOG_INFO("SYS", "Warm Start: Silent listening for %d ms...", networkScanDuration);
     } else {
         LOG_INFO("SYS", "Cold Start: Scanning for %d ms...", networkScanDuration);
     }
     
     _display.toggleLed();
     
     uint32_t scanStart = millis();
     uint32_t lastDispUpdate = 0;
     
     // Мы не можем напрямую сбросить флаг прерывания, но можем подготовить радио
     _radio.startReceive(); 
     
     // Цикл немого прослушивания радиоэфира
     while (millis() - scanStart < networkScanDuration) {
         uint32_t now = millis();
         
         // Обновляем дисплей каждую секунду
         if (now - lastDispUpdate >= 1000) {
             lastDispUpdate = now;
             uint32_t left = (networkScanDuration - (now - scanStart)) / 1000;
             
             String startTitle = "Start-" + String(_bleManager.macSuffix);
             
             // Получаем общее количество узлов и вычитаем себя (Донгл)
             uint8_t totalNodes = _nodeDB.getActiveNodesCount();
             uint8_t foundNeighbors = (totalNodes > 0) ? (totalNodes - 1) : 0;
             
             _display.showStatus(startTitle, 
                                "Time left: " + String(left) + " s", 
                                "Neighbors: " + String(foundNeighbors), 
                                "Please wait...");
         } 
 
         // Обработка входящих пакетов во время сканирования
         // ВАЖНО: В режиме сканирования мы используем прямой опрос радио
         size_t len = _radio.getPacketLength();
         if (len >= sizeof(NavigaHeader)) {
             uint8_t rxBuffer[256];             
             int state = _radio.readData(rxBuffer, len); 
             if (state == RADIOLIB_ERR_NONE) {
                 NavigaHeader rxHeader;
                 memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                 size_t payloadLen = len - sizeof(NavigaHeader);
                 
                 if (_router.isValidPacket(rxHeader.getType(), payloadLen)) {
                     if (!_router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         _packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen);
                     } 
                 } 
             } 
             _radio.startReceive();
         } 
         
         _gps.update(); // Поддерживаем опрос GPS
     } 
     
     _display.toggleLed();
     
     // Если это Cold Start (или у нас почему-то нет ID), генерируем новый
     if (!isWarmStart || myNodeId == 0) {
         randomSeed(esp_random());
         do {
             myNodeId = random(1, 255);
         } while (_nodeDB.getNode(myNodeId) != nullptr); 
         
         _nodeDB.addNode(myNodeId); 
         LOG_INFO("SYS", "Scan complete. Selected unique Node ID: %d", myNodeId);
     } else {
         LOG_INFO("SYS", "Silent listening complete. Kept Node ID: %d", myNodeId);
     }
     
     _display.showStatus("Scan Complete", "My ID:", String(myNodeId), "Starting...");
     delay(2000);
 }  //NetworkManager.cpp
```

---

## File: .\src\CoordProcessor.h
```cpp
/**
 * File: CoordProcessor.h
 * Version: 1.00 (Refactored from main.cpp v1.44)
 * Description: Выделенный модуль для гео-математики: расчет дистанций, азимутов и распаковка координат.
 */

 #ifndef COORD_PROCESSOR_H
 #define COORD_PROCESSOR_H
 
 #include <Arduino.h>
 #include "NodeDatabase.h"
 #include "GpsManager.h"
 #include "GeoPacker.h"
 
 class CoordProcessor {
 public:
     CoordProcessor();
 
     /**
      * Основной цикл обработки координат всех узлов в базе.
      * @param nodeDB Ссылка на базу узлов
      * @param gps Ссылка на менеджер GPS донгла
      * @param packer Ссылка на упаковщик координат
      * @param isFastTracker Флаг оптимизации (пропуск расчетов при движении)
      */
     void process(NodeDatabase& nodeDB, GpsManager& gps, GeoPacker& packer, bool isFastTracker);
 
 private:
     bool _isLonScaleSet;
     float _lastScaleLat;
 };
 
 #endif // COORD_PROCESSOR_H
```

---

## File: .\src\CoordProcessor.cpp
```cpp
/**
 * File: CoordProcessor.cpp
 * Version: 1.00 (Refactored from main.cpp v1.44)
 * Description: Реализация фоновых гео-вычислений. Код перенесен из main.cpp без изменений.
 */

 #include "CoordProcessor.h"
 #include "logger.h"
 
 CoordProcessor::CoordProcessor() : _isLonScaleSet(false), _lastScaleLat(0.0f) {
 }
 
 void CoordProcessor::process(NodeDatabase& nodeDB, GpsManager& gps, GeoPacker& packer, bool isFastTracker) {
     if (!gps.isValid()) return;
 
     float currentLat = gps.getLat();
     float currentLon = gps.getLon();
 
     // Динамический пересчет масштаба при смещении по широте (> 1 градуса)
     if (!_isLonScaleSet || abs(currentLat - _lastScaleLat) > 1.0f) {
         packer.updateLonScale(currentLat); // Устанавливаем масштаб по широте
         _lastScaleLat = currentLat;
         _isLonScaleSet = true;
         LOG_INFO("SYS", "Longitude scale updated for Lat: %.4f", currentLat);
     } 
 
     // Если мы не в режиме быстрого трекера, обновляем топологию всех активных узлов
     if (!isFastTracker) {
         for (int i = 1; i < 255; i++) {
             const NodeRecord* node = nodeDB.getNode(i);
             
             // Обрабатываем только активные узлы (кроме самого себя, хотя myNodeId отфильтруется логикой расстояния 0)
             if (node != nullptr && node->isActive) {
                 
                 // 1. Распаковка координат: если у нас есть сырые данные, но нет float значений
                 if (node->packedCoords != 0 && node->lat == 0.0f && node->lon == 0.0f) {
                     float unpLat, unpLon;
                     packer.unpack(node->packedCoords, currentLat, currentLon, unpLat, unpLon);
                     
                     // Обновляем координаты в базе без изменения флага активности
                     nodeDB.updateNodeCoords(i, unpLat, unpLon, node->packedCoords, false);
                 } 
                 
                 // 2. Расчет относительной топологии (Дистанция и Азимут)
                 if (node->lat != 0.0f || node->lon != 0.0f) {
                     float d = gps.distanceTo(node->lat, node->lon);
                     float a = gps.courseTo(node->lat, node->lon);
                     
                     nodeDB.updateNodeDistanceAzimuth(i, d, a);
                 } 
             } 
         } 
     }
 } //CoordProcessor.cpp
```

---

