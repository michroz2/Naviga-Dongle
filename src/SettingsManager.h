/**
 * File: SettingsManager.h
 * Version: 1.27
 * Description: Управление энергонезависимой памятью (NVS) ESP32.
 * Сохранение настроек и слепков топологии (реализация UC-03).
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
     char name[12];
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
 