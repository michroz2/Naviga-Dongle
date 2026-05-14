/**
 * Project: Naviga-Dongle
 * File: DBManager.h
 * Version: 1.43.8
 * Изменение: Асинхронная неблокирующая выгрузка базы (State Machine).
 * Description: Контроллер базы данных. Управляет идентификацией локального узла,
 * топологией сети, очисткой мусора и синхронизацией с мобильным приложением.
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
     
     // Передаем myNodeId и myMsgSeq по ссылке, чтобы они обновились в main.cpp
     void handleCollision(uint8_t& myNodeId, uint8_t& myMsgSeq, uint8_t myNodeType);
 
     // --- ТОПОЛОГИЯ И КАРТОГРАФИЯ (Глобальные задачи) ---
     void processBackgroundTasks(bool isFastTracker, uint8_t myNodeId);
     void updateGeodata(bool isFastTracker);
 
     void clearDatabase(uint8_t myNodeId);
 
 private:
     NodeDatabase& _db;
     TxManager& _tx;
     BleManager& _ble;
     SettingsManager& _settings;
     GpsManager& _gps;
     GeoPacker& _packer;
 
     // Таймеры и флаги перенесены из глобальной области видимости main.cpp
     uint32_t _lastTopologyUpdateTime;
     uint32_t _lastCleanupTime;
     
     bool _isLonScaleSet;
     float _lastScaleLat;
 };
 
 #endif // DB_MANAGER_H