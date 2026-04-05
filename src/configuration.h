/**
 * File: configuration.h
 * Version: 1.11 Изменение: Добавлен параметр DEFAULT_TTL для ограничения ретрансляции.
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

// --- НАСТРОЙКИ АДАПТИВНОЙ ОТПРАВКИ КООРДИНАТ ---
#define MIN_MOVEMENT_METERS    15.0f  // Порог дистанции для фиксации движения (защита от дрейфа)
#define MIN_SPEED_KMPH         2.0f   // Минимальная скорость для подтверждения движения (защита от дрейфа)
#define SNEAK_MOVEMENT_METERS  40.0f  // Дистанция безусловной отправки (для медленного движения/крадущегося)
#define TX_INTERVAL_MOVING     5000   // Максимальная частота отправки в движении (5 сек)
#define TX_INTERVAL_STILL      300000 // Редкий пинг на стоянке (5 минут)

// --- НАСТРОЙКИ СЕТИ ---
#define DEFAULT_TTL 3 // Максимальное количество ретрансляций (прыжков) для одного пакета

#endif // CONFIGURATION_H