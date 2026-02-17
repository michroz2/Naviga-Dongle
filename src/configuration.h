#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

// --- LED (RED) ---
#define LED_PIN     4   // Красный светодиод на плате T-Beam v1.1
#define LED_ON      LOW // Управляется низким уровнем (Active LOW)
#define LED_OFF     HIGH

// --- I2C PINS (OLED & AXP2101) ---
// На T-Beam v1.1 шина I2C жестко задана:
#define I2C_SDA 21
#define I2C_SCL 22

// --- POWER MANAGEMENT (AXP2101) ---
// Адрес I2C для PMU
#define PMU_IRQ 35  // Внимание! Этот пин часто делит прерывание с LoRa DIO1 на старых ревизиях, проверим.
                    // НО! Так как у нас внешняя LoRa на кастомных пинах, конфликта не будет.

// --- EBYTE E22-400M33S (SX1268) PINOUT ---
// Твоя кастомная разводка:

// изменено begin
#undef LORA_CS
#undef LORA_RST
#undef LORA_BUSY
#undef LORA_DIO1

#define LORA_CS     13  // Chip Select
#define LORA_RST    14  // Reset
#define LORA_BUSY   15  // Busy (Critical for SX126x)
#define LORA_DIO1   35  // DIO1 (Interrupt) - Пин 35 на ESP32 только вход (Input Only)
#define LORA_TXEN   2   // TX Enable (и он же управляет синим светодиодом на плате)
#define LORA_RXEN   25  // RX Enable

// --- GPS (Ublox NEO-8M) ---
// Стандартные пины T-Beam v1.1:
#define GPS_RX      34
#define GPS_TX      12

#endif