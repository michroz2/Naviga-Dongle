/**
 * File: configuration.h
 * Version: 1.20 Изменение: Добавлены флаги условной компиляции интерфейса (HAS_DISPLAY, HAS_STATUS_LED).
 * Description: Конфигурация пинов и базовых настроек.
 */
 
 #ifndef CONFIGURATION_H
 #define CONFIGURATION_H
 
 #include <Arduino.h>
 
 // --- ОПЦИИ ИНТЕРФЕЙСА (Рефакторинг UI) ---
 #define HAS_DISPLAY     1  // 1 - включить код OLED дисплея, 0 - вырезать
 #define HAS_STATUS_LED  1  // 1 - включить код статус-светодиода, 0 - вырезать
 
 // --- LED (RED) ---
 #define LED_PIN     4   
 #define LED_ON      LOW 
 #define LED_OFF     HIGH 
 
 // --- I2C PINS (OLED & AXP2101) ---
 #define I2C_SDA 21      
 #define I2C_SCL 22      
 
 // --- POWER MANAGEMENT (AXP2101) ---
 #define PMU_IRQ 35      
 
 // --- LORA (ONBOARD SX1276) ---
 #define LORA_ONBOARD_CS 18
 
 // --- EBYTE E22-400M33S (SX1268) PINOUT ---
 #undef LORA_CS
 #undef LORA_RST
 #undef LORA_BUSY
 #undef LORA_DIO1
 #undef LORA_SCK
 #undef LORA_MISO
 #undef LORA_MOSI
 
 #define LORA_CS     13  
 #define LORA_RST    14  
 #define LORA_BUSY   15  
 #define LORA_DIO1   35  
 #define LORA_TXEN   2   
 #define LORA_RXEN   25  
 
 // --- SPI BUS ---
 #define LORA_SCK    5   
 #define LORA_MISO   19  
 #define LORA_MOSI   27  
 
 // --- GPS (Ublox NEO-8M) ---
 #define GPS_RX      34  
 #define GPS_TX      12  
 
 // --- ТАЙМЕРЫ И ИНТЕРВАЛЫ ---
 const uint32_t txInterval = 10000;
 const uint32_t gpsUpdateInterval = 1000;
 
 #define CLEANUP_INTERVAL_MS 10000       
 #define HEARTBEAT_INTERVAL_MS 2700000   
 #define MIN_GREETING_NODEINFO_JITTER 120000
 #define MAX_GREETING_NODEINFO_JITTER 300000
 
 // --- НАСТРОЙКИ АДАПТИВНОЙ ОТПРАВКИ КООРДИНАТ ---
 #define MIN_MOVEMENT_METERS    15.0f  // Порог дистанции для фиксации движения (защита от дрейфа)
 #define MIN_SPEED_KMPH         2.0f   // Минимальная скорость для подтверждения движения (защита от дрейфа)
 #define SNEAK_MOVEMENT_METERS  40.0f  // Дистанция безусловной отправки (для медленного движения/крадущегося)
 #define TX_INTERVAL_MOVING     5000   // Максимальная частота отправки в движении (5 сек)
 #define TX_INTERVAL_STILL      300000 // Редкий пинг на стоянке (5 минут)
 
 // --- НАСТРОЙКИ СЕТИ И РОЛЕЙ ---
 #define DEFAULT_TTL 3 
 #define MAX_DIRECT_CONNECT_METERS 200.0f 
 #define MIN_RELAY_DISTANCE_METERS 20.0f  // Минимальная дистанция до узла для участия в векторной ретрансляции
 #define TOPOLOGY_UPDATE_INTERVAL_MS 15000 
 #define TRACKER_FAST_SPEED_KMPH 5.0f     // Порог скорости (Бег/Езда) для отключения ретрансляции у Трекера
 
 // --- СТАТИЧЕСКАЯ ПОЗИЦИЯ И ДЖИТТЕР (РОЛИ) ---
 #define RELAY_STATIC_LAT 0.0f
 #define RELAY_STATIC_LON 0.0f
 
 #define RELAY_JITTER_MIN_MS 100
 #define RELAY_JITTER_MAX_MS 600
 #define STALKER_JITTER_MIN_MS 300
 #define STALKER_JITTER_MAX_MS 1000
 
 #endif // CONFIGURATION_H