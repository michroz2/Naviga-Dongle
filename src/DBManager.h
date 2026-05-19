/**
 * Project: Naviga-Dongle
 * File: DBManager.h
 * Version: 1.47.2
 * Изменение: Изменена сигнатура updateGeodata для поддержки работы с собственным ID.
 * Description: Заголовочный файл контроллера базы данных узлов.
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
 #include "configuration.h"
 
 class DBManager {
 public:
      DBManager(NodeDatabase& db, TxManager& tx, BleManager& ble,
                SettingsManager& settings, GpsManager& gps, GeoPacker& packer);
 
      uint8_t generateUniqueId();
      void handleCollision(uint8_t& myNodeId, uint8_t& myMsgSeq, uint8_t myNodeType);
      void processBackgroundTasks(bool isFastTracker, uint8_t myNodeId);
      
      // ИЗМЕНЕНИЕ 1.47.2: Добавлен myNodeId для обновления собственных координат
      void updateGeodata(bool isFastTracker, uint8_t myNodeId); 
      
      void clearDatabase(uint8_t myNodeId);
      void setAnchor(float lat, float lon);
 
 private:
      NodeDatabase& _db;
      TxManager& _tx;
      BleManager& _ble;
      SettingsManager& _settings;
      GpsManager& _gps;
      GeoPacker& _packer;
 
      uint32_t _lastTopologyUpdateTime;
      uint32_t _lastCleanupTime;
 
      bool _isLonScaleSet;
      float _lastScaleLat;
 };
 
 #endif // DB_MANAGER_H