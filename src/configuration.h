#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

// --- LED (RED) ---
#define LED_PIN     4   // Красный светодиод на плате T-Beam v1.1
#define LED_ON      LOW // Активный уровень: LOW
#define LED_OFF     HIGH

// --- I2C PINS (OLED & AXP2101) ---
// Аппаратная шина I2C T-Beam v1.1
#define I2C_SDA 21
#define I2C_SCL 22

// --- POWER MANAGEMENT (AXP2101) ---
// Адрес I2C определяется программно, здесь задан пин прерывания
#define PMU_IRQ 35  // Пин прерывания PMU. На старых ревизиях может совпадать с DIO1.
                    // В данной конфигурации конфликт отсутствует (DIO1 вынесен).

// --- LORA (ONBOARD SX1276) ---
// Пин Chip Select встроенного модуля. Требуется высокий уровень (HIGH) для отключения.
#define LORA_ONBOARD_CS 18

// --- EBYTE E22-400M33S (SX1268) PINOUT ---
// Сброс определений SPI перед явным заданием
#undef LORA_CS
#undef LORA_RST
#undef LORA_BUSY
#undef LORA_DIO1
#undef LORA_SCK
#undef LORA_MISO
#undef LORA_MOSI

// Кастомная конфигурация пинов для модуля E22
#define LORA_CS     13  // Chip Select
#define LORA_RST    14  // Reset
#define LORA_BUSY   15  // Busy Line (Критичен для SX126x)
#define LORA_DIO1   35  // DIO1 (Interrupt). GPIO35 — только вход (Input Only).
#define LORA_TXEN   2   // TX Enable (Дублируется на синий светодиод)
#define LORA_RXEN   25  // RX Enable

// --- SPI BUS ---
// Стандартные пины SPI для T-Beam v1.1, используемые для подключения E22
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27

// --- GPS (Ublox NEO-8M) ---
// UART интерфейс GPS модуля
#define GPS_RX      34
#define GPS_TX      12

// --- НАСТРОЙКИ ПЕРИОДИЧНОСТИ ---
uint32_t txInterval = 15000;      // Интервал передачи координат (мс)
uint32_t gpsUpdateInterval = 1000; // Интервал обновления GPS данных (мс)

#endif