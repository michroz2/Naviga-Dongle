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
 #define VERSION_STR "1.46.5"
 
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