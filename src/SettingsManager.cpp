/**
 * File: SettingsManager.cpp
 * Version: 1.39
 * Изменение: Реализован метод factoryReset() для стирания всех данных и возврата к заводским настройкам (UC-08).
 * Description: Реализация класса управления энергонезависимой памятью (NVS).
 */

 #include "SettingsManager.h"
 #include "NodeDatabase.h"
 #include "logger.h"
 
 // Глобальный экземпляр
 SettingsManager settingsManager;
 
 SettingsManager::SettingsManager() {
     // Конструктор пуст, инициализация в init()
 } // SettingsManager::SettingsManager()
 
 void SettingsManager::init() {
     _preferences.begin(PREFS_NAMESPACE, false); // Открываем NVS в режиме чтения/записи
     
     if (_preferences.isKey(PREFS_CFG_KEY)) {
         // Загружаем настройки, если они уже есть в памяти
         _preferences.getBytes(PREFS_CFG_KEY, &settings, sizeof(NavigaSettings));
         LOG_INFO("SYS", "Settings loaded from NVS. Node ID: %d", settings.nodeId);
     } else {
         // Первый запуск - загружаем дефолты
         LOG_INFO("SYS", "No settings found in NVS. Loading defaults.");
         loadDefaults();
         save(); // Сразу сохраняем структуру, чтобы ключ появился
     } // if (_preferences.isKey(PREFS_CFG_KEY))
     
     _preferences.end();
 } // SettingsManager::init()
 
 void SettingsManager::loadDefaults() {
     settings.nodeId = DEFAULT_NODE_ID;
     settings.nodeType = DEFAULT_NODE_TYPE;
     
     // Безопасное копирование дефолтного имени
     strncpy(settings.nodeName, DEFAULT_NODE_NAME, sizeof(settings.nodeName) - 1);
     settings.nodeName[sizeof(settings.nodeName) - 1] = '\0';
     
     settings.txIntervalMoving = DEFAULT_TX_INTERVAL_MOVING;
     settings.txIntervalStill = DEFAULT_TX_INTERVAL_STILL;
     settings.nodeConnectionTimeout = DEFAULT_NODE_CONN_TIMEOUT;
     settings.nodeActiveTimeoutMs = DEFAULT_NODE_ACTIVE_TIMEOUT;
     
     settings.isConfigured = false;
 } // SettingsManager::loadDefaults()
 
 void SettingsManager::save() {
     _preferences.begin(PREFS_NAMESPACE, false);
     _preferences.putBytes(PREFS_CFG_KEY, &settings, sizeof(NavigaSettings));
     _preferences.end();
 } // SettingsManager::save()
 
 // ИЗМЕНЕНИЕ 1.39: Полная очистка памяти (Factory Reset)
 void SettingsManager::factoryReset() {
     _preferences.begin(PREFS_NAMESPACE, false);
     _preferences.clear(); // Удаляет все ключи (и настройки, и слепки узлов) в пространстве naviga
     _preferences.end();
     
     loadDefaults(); // Сбрасываем структуру в оперативной памяти
     LOG_INFO("SYS", "Factory Reset completed. Flash memory cleared.");
 } // SettingsManager::factoryReset()
 
 void SettingsManager::saveNodesSnapshot(const NodeDatabase& db) {
     SavedNodeRecord snapshot[50]; // Лимит в 50 узлов для слепка
     uint8_t count = 0;
     
     for (int i = 1; i < 255; i++) {
         const NodeRecord* node = db.getNode(i);
         if (node != nullptr && node->isActive) {
             snapshot[count].id = node->nodeId;
             snapshot[count].role = node->type;
             strncpy(snapshot[count].name, node->nodeName, sizeof(snapshot[count].name) - 1);
             snapshot[count].name[sizeof(snapshot[count].name) - 1] = '\0';
             snapshot[count].packedCoords = node->packedCoords;
             
             count++;
             if (count >= 50) break; // Защита от переполнения выделенного массива
         } // if (node != nullptr && node->isActive)
     } // for (int i = 1; i < 255; i++)
     
     if (count > 0) {
         _preferences.begin(PREFS_NAMESPACE, false);
         _preferences.putBytes(PREFS_NODES_KEY, snapshot, count * sizeof(SavedNodeRecord));
         _preferences.end();
         LOG_INFO("SYS", "Saved snapshot of %d nodes to NVS.", count);
     } // if (count > 0)
 } // SettingsManager::saveNodesSnapshot()
 
 void SettingsManager::loadNodesSnapshot(NodeDatabase& db) {
     _preferences.begin(PREFS_NAMESPACE, true); // Открываем только для чтения
     
     if (_preferences.isKey(PREFS_NODES_KEY)) {
         size_t dataLen = _preferences.getBytesLength(PREFS_NODES_KEY);
         uint8_t count = dataLen / sizeof(SavedNodeRecord);
         
         if (count > 0 && count <= 50) {
             SavedNodeRecord snapshot[50];
             _preferences.getBytes(PREFS_NODES_KEY, snapshot, dataLen);
             
             for (uint8_t i = 0; i < count; i++) {
                 db.addNode(snapshot[i].id);
                 db.updateNodeInfo(snapshot[i].id, snapshot[i].name, snapshot[i].role);
                 db.updateNodeCoords(snapshot[i].id, 0.0f, 0.0f, snapshot[i].packedCoords, true);
             } // for (uint8_t i = 0; i < count; i++)
             LOG_INFO("SYS", "Loaded snapshot of %d nodes from NVS.", count);
         } // if (count > 0 && count <= 50)
     } // if (_preferences.isKey(PREFS_NODES_KEY))
     
     _preferences.end();
 } // SettingsManager::loadNodesSnapshot()