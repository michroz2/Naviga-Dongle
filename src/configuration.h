/**
 * File: configuration.h
 * Version: 1.27 
 * Изменение: Перенос пользовательских таймеров и ролей в структуру NavigaSettings (UC-03).
 * Description: Конфигурация пинов и базовых настроек для разных аппаратных платформ.
 */
 
 #ifndef CONFIGURATION_H
 #define CONFIGURATION_H
 
 #include <Arduino.h>
 
 // ==========================================================
 // --- ВЫБОР ПЛАТЫ (Должен задаваться в platformio.ini) ---
 // ==========================================================
 #if !defined(BOARD_T_BEAM_V11) && !defined(BOARD_T_ENERGY_S3)
     #define BOARD_T_BEAM_V11 
 #endif
 
 // ==========================================================
 // СЕКЦИЯ 1: Конфигурация ESP32 T-Beam v1.1 (Оригинал)
 // ==========================================================
 #ifdef BOARD_T_BEAM_V11
     #define BOARD_NAME "T-Beam v1.1"
 
     // Опции аппаратного обеспечения
     #define HAS_PMU 1               // Есть чип питания AXP2101
     #define LORA_CHIP_SX1268        // Используется чип SX1268 (E22)
 
     // --- LED (RED) ---
     #define LED_PIN     4   
     #define LED_ON      LOW 
     #define LED_OFF     HIGH 
 
     // --- I2C PINS (OLED & AXP2101) ---
     #define I2C_SDA     21      
     #define I2C_SCL     22      
 
     // --- POWER MANAGEMENT (AXP2101) ---
     #define PMU_IRQ     35      
 
     // --- LORA (ONBOARD SX1276) - Отключен аппаратно CS ---
     #define LORA_ONBOARD_CS 18
 
     // Отменяем стандартные определения пинов T-Beam из системного pins_arduino.h,
     // чтобы компилятор не выдавал предупреждения "redefined"
     #undef LORA_CS
     #undef LORA_RST
     #undef LORA_IRQ
 
     // --- EBYTE E22-400M33S (SX1268) PINOUT ---
     #define LORA_CS     13  
     #define LORA_RST    14  
     #define LORA_BUSY   15  
     #define LORA_IRQ    35  // DIO1
     #define LORA_TXEN   2   
     #define LORA_RXEN   25  
 
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
     #define HAS_PMU 0               // Питание прямое, AXP2101 отсутствует
     #define LORA_CHIP_SX1268        
 
     // Стандартный LED для S3 (может варьироваться от ревизии)
     #define LED_PIN     0   
     #define LED_ON      HIGH 
     #define LED_OFF     LOW
 
     // --- DISPLAY OLED I2C PINS --- 
     #define I2C_SDA     5 
     #define I2C_SCL     4 
     
     #define PMU_IRQ     -1      
     #define LORA_ONBOARD_CS -1 
 
     // --- LORA E22-400M33S (SX1268) PINOUT (v1.25) ---
     #define LORA_CS     9   // NSS
     #define LORA_RST    13  // NRST
     #define LORA_BUSY   14  
     #define LORA_IRQ    21  // DIO1
     #define LORA_RXEN   7   
     #define LORA_TXEN   6   
 
     // --- SPI BUS (Hardware FSPI via GPIO Matrix) ---
     #define LORA_SCK    10  
     #define LORA_MISO   12  
     #define LORA_MOSI   11  
 
     // --- GPS (GY-GPS6MV2 / NEO-6M) ---
     // RX контроллера (18) подключен к TX модуля GPS
     #define GPS_RX      18  
     #define GPS_TX      17  
 #endif
 
 // ==========================================================
 // СЕКЦИЯ 3: Глобальные настройки (Общие для всех плат)
 // ==========================================================
 
 // --- ОПЦИИ ИНТЕРФЕЙСА (Рефакторинг UI v1.20) ---
 #define HAS_DISPLAY     1  // 1 - включить код OLED дисплея, 0 - вырезать
 #define HAS_STATUS_LED  1  // 1 - включить код статус-светодиода, 0 - вырезать
 
 // --- ТАЙМЕРЫ И ИНТЕРВАЛЫ ---
 const uint32_t txInterval = 10000;
 const uint32_t gpsUpdateInterval = 1000;
 
 #define CLEANUP_INTERVAL_MS 10000       
 #define HEARTBEAT_INTERVAL_MS 2700000   
 #define MIN_GREETING_NODEINFO_JITTER 120000
 #define MAX_GREETING_NODEINFO_JITTER 300000
 
 // --- НАСТРОЙКИ АДАПТИВНОЙ ОТПРАВКИ КООРДИНАТ (v1.18) ---
 #define MIN_MOVEMENT_METERS    15.0f  // Порог дистанции для фиксации движения (защита от дрейфа)
 #define MIN_SPEED_KMPH         2.0f   // Минимальная скорость для подтверждения движения (защита от дрейфа)
 #define SNEAK_MOVEMENT_METERS  40.0f  // Дистанция безусловной отправки (для медленного движения/крадущегося)
 
 // --- НАСТРОЙКИ СЕТИ И РОЛЕЙ ---
 #define DEFAULT_TTL 3 
 #define MAX_DIRECT_CONNECT_METERS 200.0f 
 #define MIN_RELAY_DISTANCE_METERS 20.0f  // Минимальная дистанция до узла для участия в векторной ретрансляции
 #define TOPOLOGY_UPDATE_INTERVAL_MS 15000 
 #define TRACKER_FAST_SPEED_KMPH 5.0f     // Порог скорости (Бег/Езда) для отключения ретрансляции у Трекера (v1.18)
 
 // --- СТАТИЧЕСКАЯ ПОЗИЦИЯ И ДЖИТТЕР (РОЛИ v1.19) ---
 #define RELAY_STATIC_LAT 0.0f
 #define RELAY_STATIC_LON 0.0f
 
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
 #define DEFAULT_TX_INTERVAL_MOVING    5000         // 5 сек
 #define DEFAULT_TX_INTERVAL_STILL     300000       // 5 минут
 #define DEFAULT_NODE_CONN_TIMEOUT     30000        // 30 сек (потеря связи)
 #define DEFAULT_NODE_ACTIVE_TIMEOUT   600000       // 10 минут (очистка из базы)

 struct NavigaSettings {
     uint8_t nodeId;
     uint8_t nodeType;
     char nodeName[12];
     uint32_t txIntervalMoving;
     uint32_t txIntervalStill;
     uint32_t nodeConnectionTimeout; 
     uint32_t nodeActiveTimeoutMs;   
     bool isConfigured; 
 };
 
 #endif // CONFIGURATION_H