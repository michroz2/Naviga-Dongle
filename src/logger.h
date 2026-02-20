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