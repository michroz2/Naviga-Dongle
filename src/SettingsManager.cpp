/**
 * File: SettingsManager.cpp
 * Version: 1.27
 * Description: Реализация сохранения настроек и слепка топологии в NVS память ESP32.
 */

 #include "SettingsManager.h"
 #include "logger.h"
 #include "NodeDatabase.h"
 
 SettingsManager settingsManager;
 
 SettingsManager::SettingsManager() {}
 
 void SettingsManager::init() {
     _preferences.begin(PREFS_NAMESPACE, false); // false = режим Read/Write
     
     // Если по ключу конфигурации нет данных (первое включение или сброс)
     if (!_preferences.isKey(PREFS_CFG_KEY)) {
         LOG_INFO("SYS", "NVS: No saved settings found. Loading defaults.");
         loadDefaults();
         save(); 
     } else {
         // Чтение байтов в структуру settings
         size_t len = _preferences.getBytes(PREFS_CFG_KEY, &settings, sizeof(NavigaSettings));
         // Валидация размера: если версия структуры изменилась, сбросить настройки
         if (len == sizeof(NavigaSettings)) {
             LOG_INFO("SYS", "NVS: Settings loaded successfully.");
             LOG_INFO("SYS", "-> Configured: %s", settings.isConfigured ? "YES" : "NO");
             LOG_INFO("SYS", "-> Node ID: %d, Role: %d", settings.nodeId, settings.nodeType);
         } else {
             LOG_WARN("SYS", "NVS: Settings size mismatch! Resetting to defaults.");
             loadDefaults();
             save();
         }
     }
     _preferences.end(); // Закрываем доступ к NVS
 }
 
 void SettingsManager::loadDefaults() {
     settings.nodeId = DEFAULT_NODE_ID;
     settings.nodeType = DEFAULT_NODE_TYPE;
     strncpy(settings.nodeName, DEFAULT_NODE_NAME, sizeof(settings.nodeName) - 1);
     settings.nodeName[sizeof(settings.nodeName) - 1] = '\0';
     
     settings.txIntervalMoving = DEFAULT_TX_INTERVAL_MOVING;
     settings.txIntervalStill = DEFAULT_TX_INTERVAL_STILL;
     settings.nodeConnectionTimeout = DEFAULT_NODE_CONN_TIMEOUT;
     settings.nodeActiveTimeoutMs = DEFAULT_NODE_ACTIVE_TIMEOUT;
     
     settings.isConfigured = false;
 }
 
 void SettingsManager::save() {
     _preferences.begin(PREFS_NAMESPACE, false);
     // Сериализация структуры в бинарный блок и запись в память
     size_t len = _preferences.putBytes(PREFS_CFG_KEY, &settings, sizeof(NavigaSettings));
     _preferences.end();
     
     if (len > 0) {
         LOG_INFO("SYS", "NVS: Settings successfully saved to flash memory.");
     } else {
         LOG_WARN("SYS", "NVS: Failed to save settings!");
     }
 }
 
 void SettingsManager::factoryReset() {
     _preferences.begin(PREFS_NAMESPACE, false);
     _preferences.clear(); // Полная очистка всех ключей в пространстве имен
     _preferences.end();
     LOG_INFO("SYS", "NVS: Factory reset completed.");
     loadDefaults();
 }
 
 // Создание и сохранение "Снимка" локальной базы соседей (ограничено до 50 узлов)
 void SettingsManager::saveNodesSnapshot(const NodeDatabase& db) {
     SavedNodeRecord snapshot[50]; 
     size_t count = 0;
     
     for(int i = 1; i < 255 && count < 50; i++) {
         const NodeRecord* node = db.getNode(i);
         if(node != nullptr && node->isActive) {
             snapshot[count].id = node->nodeId;
             snapshot[count].role = node->type;
             strncpy(snapshot[count].name, node->nodeName, sizeof(snapshot[count].name) - 1);
             snapshot[count].name[sizeof(snapshot[count].name) - 1] = '\0';
             snapshot[count].packedCoords = node->packedCoords; // Сохраняем сырые сжатые координаты
             count++;
         }
     }
     
     // Если есть что сохранять, сериализуем массив во Flash
     if (count > 0) {
         _preferences.begin(PREFS_NAMESPACE, false);
         _preferences.putBytes(PREFS_NODES_KEY, snapshot, count * sizeof(SavedNodeRecord));
         _preferences.end();
         LOG_INFO("SYS", "NVS: Saved topology snapshot (Nodes: %d)", count);
     }
 }
 
 // Восстановление снимка соседей при "Горячем старте" (Warm Start)
 void SettingsManager::loadNodesSnapshot(NodeDatabase& db) {
     _preferences.begin(PREFS_NAMESPACE, true); // true = режим Read Only
     size_t len = _preferences.getBytesLength(PREFS_NODES_KEY);
     
     // Проверка валидности длины массива (кратно структуре SavedNodeRecord)
     if(len == 0 || len % sizeof(SavedNodeRecord) != 0) {
         _preferences.end();
         return;
     }
     
     size_t count = len / sizeof(SavedNodeRecord);
     // Динамически выделяем буфер для чтения (чтобы не забивать стек)
     SavedNodeRecord* snapshot = new SavedNodeRecord[count];
     _preferences.getBytes(PREFS_NODES_KEY, snapshot, len);
     _preferences.end();
 
     // Восстанавливаем узлы в базу (nodeDB)
     for(size_t i = 0; i < count; i++) {
         db.addNode(snapshot[i].id);
         db.updateNodeInfo(snapshot[i].id, snapshot[i].name, snapshot[i].role);
         // Загружаем packedCoords. updateNodeCoords обновляет lastSeen внутри себя
         db.updateNodeCoords(snapshot[i].id, 0.0f, 0.0f, snapshot[i].packedCoords, false);
     }
     
     delete[] snapshot; // Освобождаем память
     LOG_INFO("SYS", "NVS: Loaded topology snapshot (Nodes: %d)", count);
 } //SettingsManager.cpp
 