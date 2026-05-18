/**
 * Project: Naviga-Dongle
 * File: DBManager.h
 * Version: 1.46.7
 * Description: Контроллер базы данных. Управляет идентификацией локального узла,
 * топологией сети и очисткой мусора.
 * Изменение: Добавлен метод setAnchor для проброса координат Оператора в GpsManager.
 */

 #ifndef DB_MANAGER_H
 #define DB_MANAGER_H
 
 #include <Arduino.h>
 #include "NodeDatabase.h"
 #include "TxManager.h"
 #include "BleManager.h"
 #include "SettingsManager.h"
 #include "GpsManager.h"
 #include "GeoPacker.h"
 #include "logger.h"
 
 class DBManager {
 public:
     DBManager(NodeDatabase& db, TxManager& tx, BleManager& ble,
               SettingsManager& settings, GpsManager& gps, GeoPacker& packer);
 
     // --- ИДЕНТИФИКАЦИЯ (Локальный узел) ---
     uint8_t generateUniqueId();
     
     // Обработка коллизий с уведомлением Оператора
     void handleCollision(uint8_t& myNodeId, uint8_t& myMsgSeq, uint8_t myNodeType);
 
     // --- ТОПОЛОГИЯ И КАРТОГРАФИЯ (Глобальные задачи) ---
     void processBackgroundTasks(bool isFastTracker, uint8_t myNodeId);
     void updateGeodata(bool isFastTracker);
 
     // --- УПРАВЛЕНИЕ БАЗОЙ ---
     void clearDatabase(uint8_t myNodeId);
     
     // НОВОЕ 1.46.7: Шлюз для проброса опорной точки в GpsManager
     void setAnchor(float lat, float lon);
 
 private:
     NodeDatabase& _db;
     TxManager& _tx;
     BleManager& _ble;
     SettingsManager& _settings;
     GpsManager& _gps;
     GeoPacker& _packer;
 
     // Системные таймеры
     uint32_t _lastTopologyUpdateTime;
     uint32_t _lastCleanupTime;
     
     bool _isLonScaleSet;
     float _lastScaleLat;
 };
 
 #endif // DB_MANAGER_H