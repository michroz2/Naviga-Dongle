/**
 * File: configuration.h
 * Version: 1.22  -Energy S3 переведен на модуль E22-400M33S (SX1268). Обновлена распиновка.
 * 1.23 Обновлена распиновка T-Energy S3: добавлен I2C дисплей и исправлены пины GPS.
 * Description: Конфигурация пинов и базовых настроек.
 */
 
 #ifndef CONFIGURATION_H
 #define CONFIGURATION_H
 
 #include <Arduino.h>
 
 // ==========================================================
 // --- ВЫБОР ПЛАТЫ (Должен задаваться в platformio.ini) ---
 // Если флаг не передан компилятором, по умолчанию собираем для T-Beam
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
     #define I2C_SDA 21      
     #define I2C_SCL 22      
 
     // --- POWER MANAGEMENT (AXP2101) ---
     #define PMU_IRQ 35      
 
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
     #define LORA_IRQ    35  // Для SX1268 прерывание идет через DIO1
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
 // СЕКЦИЯ 2: Конфигурация Lilygo T-Energy S3 + E22-400M33S + GPS
 // ==========================================================
 #ifdef BOARD_T_ENERGY_S3
     #define BOARD_NAME "T-Energy S3"
 
     // Опции аппаратного обеспечения
     #define HAS_PMU 0               // Питание прямое, AXP2101 отсутствует
     #define LORA_CHIP_SX1268        // ИЗМЕНЕНИЕ 1.22: Используется чип SX1268 (E22)
 
     // --- LED ---
     // На T-Energy встроенный светодиод может быть на другом пине, временно ставим заглушку
     #define LED_PIN     0   
     #define LED_ON      HIGH 
     #define LED_OFF     LOW
 
     // --- DISPLAY I2C PINS --- 
     #define I2C_SDA 4 
     #define I2C_SCL 5 
     
     // Заглушки для PMU и Onboard LoRa (чтобы код компилировался без ошибок)
     #define PMU_IRQ -1      
     #define LORA_ONBOARD_CS -1 
 
     // --- LORA E22-400M33S (SX1268) PINOUT ---
     #define LORA_CS     10  // FSPI CS0
     #define LORA_RST    9   
     #define LORA_BUSY   14  // Новый обязательный пин
     #define LORA_IRQ    21  // Основной пин прерываний (заменяет DIO0, подключается к DIO1 модуля)
     #define LORA_RXEN   7   // Управление режимом приема
     #define LORA_TXEN   8   // Управление режимом передачи
 
     // --- SPI BUS (FSPI) ---
     #define LORA_SCK    12  // FSPI CLK
     #define LORA_MISO   13  // FSPI Q
     #define LORA_MOSI   11  // FSPI D
 
     // --- GPS (GY-GPS6MV2 / NEO-6M) ---
     #define GPS_RX      18  
     #define GPS_TX      17  
 #endif
 
 // ==========================================================
 // СЕКЦИЯ 3: Глобальные настройки (Общие для всех плат)
 // ==========================================================
 
 // --- ОПЦИИ ИНТЕРФЕЙСА (Рефакторинг UI) ---
 #define HAS_DISPLAY     1  // 1 - включить код OLED дисплея, 0 - вырезать
 #define HAS_STATUS_LED  1  // 1 - включить код статус-светодиода, 0 - вырезать
 
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