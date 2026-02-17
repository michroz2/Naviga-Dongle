#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// Цветовые коды ANSI
#define LOG_COLOR_RESET  "\x1b[0m"
#define LOG_COLOR_INFO   "\x1b[32m"  // Зеленый
#define LOG_COLOR_WARN   "\x1b[33m"  // Желтый
#define LOG_COLOR_ERROR  "\x1b[31m"  // Красный
#define LOG_COLOR_DEBUG  "\x1b[36m"  // Циан

// Макросы для удобного вывода
#define LOG_INFO(tag, fmt, ...)  Serial.printf(LOG_COLOR_INFO "[INFO][%s] " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)
#define LOG_WARN(tag, fmt, ...)  Serial.printf(LOG_COLOR_WARN "[WARN][%s] " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)
#define LOG_ERROR(tag, fmt, ...) Serial.printf(LOG_COLOR_ERROR "[ERR ][%s] " fmt LOG_COLOR_RESET "\n", tag, ##__VA_ARGS__)

#endif