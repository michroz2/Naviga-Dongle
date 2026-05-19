/**
 * Project: Naviga-Dongle
 * File: DBManager.cpp
 * Version: 1.47.2
 * Description: Реализация логики контроллера базы данных узлов.
 * Изменение: Внедрена запись координат MyID в базу и защита от вычисления дистанции до себя.
 */

 #include "DBManager.h"
 #include "logger.h"
 
 DBManager::DBManager(NodeDatabase& db, TxManager& tx, BleManager& ble,
                      SettingsManager& settings, GpsManager& gps, GeoPacker& packer)
      : _db(db), _tx(tx), _ble(ble), _settings(settings), _gps(gps), _packer(packer) 
 {
      _lastTopologyUpdateTime = 0;
      _lastCleanupTime = 0;
      _isLonScaleSet = false;
      _lastScaleLat = 0.0f;
 }
 
 uint8_t DBManager::generateUniqueId() {
      uint8_t newId;
      randomSeed(esp_random());
      do {
          newId = random(1, 255);
      } while (_db.getNode(newId) != nullptr);
      return newId;
 }
 
 void DBManager::handleCollision(uint8_t& myNodeId, uint8_t& myMsgSeq, uint8_t myNodeType) {
      uint8_t oldId = myNodeId;
      
      // ИЗМЕНЕНИЕ 1.47.2: Читаем старые координаты перед удалением записи
      float oldLat = 0.0f, oldLon = 0.0f;
      uint32_t oldPacked = 0;
      const NodeRecord* oldNode = _db.getNode(oldId);
      if (oldNode != nullptr) {
          oldLat = oldNode->lat;
          oldLon = oldNode->lon;
          oldPacked = oldNode->packedCoords;
      }
      
      _db.removeNode(oldId);
 
      myNodeId = generateUniqueId();
      _db.addNode(myNodeId);
      
      // ИЗМЕНЕНИЕ 1.47.2: Восстанавливаем координаты для нового ID
      _db.updateNodeCoords(myNodeId, oldLat, oldLon, oldPacked, true);
 
      LOG_WARN("COLLISION", "ID %d is taken! Switched to new ID: %d", oldId, myNodeId);
 
      myMsgSeq = 0; 
 
      char myName[24]; 
      snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
 
      _tx.sendNodeInfo(myName, myNodeType, TX_CRITICAL);
      _db.updateNodeInfo(myNodeId, myName, myNodeType);
 
      _settings.settings.nodeId = myNodeId;
      _settings.save();
      _settings.saveNodesSnapshot(_db);
 
      _ble.sendIdentity(myNodeId, myName, myNodeType);
      LOG_INFO("BLE", "Notified App about forced ID change to %d", myNodeId);
 }
 
 void DBManager::processBackgroundTasks(bool isFastTracker, uint8_t myNodeId) {
      uint32_t currentMillis = millis();
 
      if (_gps.hasAnchor() && (currentMillis - _lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS)) {
          if (isFastTracker) {
              LOG_INFO("SYS", "Topology sync skipped: Tracker is running.");
          } else {
              _db.updateTopology();
          }
          _lastTopologyUpdateTime = currentMillis;
      }
 
      if (currentMillis - _lastCleanupTime > CLEANUP_INTERVAL_MS) {
          _db.cleanup(myNodeId);
          _lastCleanupTime = currentMillis;
      }
 }
 
 // ИЗМЕНЕНИЕ 1.47.2: Добавлен myNodeId в параметры
 void DBManager::updateGeodata(bool isFastTracker, uint8_t myNodeId) {
      if (_gps.hasAnchor()) {
          float currentLat = _gps.getLat(); 
          float currentLon = _gps.getLon();
          
          if (!_isLonScaleSet || abs(currentLat - _lastScaleLat) > 1.0f) {
              _packer.updateLonScale(currentLat);
              _lastScaleLat = currentLat;
              _isLonScaleSet = true;
              LOG_INFO("SYS", "Longitude scale updated for Lat: %.4f", currentLat);
          }
 
          // ИЗМЕНЕНИЕ 1.47.2: Записываем актуальные координаты (от GPS или Анкора) в свою базу
          uint32_t myPacked = _packer.pack(currentLat, currentLon);
          _db.updateNodeCoords(myNodeId, currentLat, currentLon, myPacked, true);
 
          if (!isFastTracker) {
              for (int i = 1; i < 255; i++) {
                  // ИЗМЕНЕНИЕ 1.47.2: Жестко отсекаем вычисление дистанции до самого себя
                  if (i == myNodeId) continue; 
                  
                  const NodeRecord *node = _db.getNode(i);
                  if (node != nullptr && node->isActive) {
                      
                      if (node->packedCoords != 0 && node->lat == 0.0f && node->lon == 0.0f) {
                          float unpLat, unpLon;
                          _packer.unpack(node->packedCoords, _gps.getLat(), _gps.getLon(), unpLat, unpLon);
                          _db.updateNodeCoords(i, unpLat, unpLon, node->packedCoords, false);
                      }
 
                      if (node->lat != 0.0f || node->lon != 0.0f) {
                          float d = _gps.distanceTo(node->lat, node->lon);
                          float a = _gps.courseTo(node->lat, node->lon);
                          _db.updateNodeDistanceAzimuth(i, d, a);
                      }
                  }
              }
          }
      }
 }
 
 void DBManager::clearDatabase(uint8_t myNodeId) {
      for (int i = 1; i < 255; i++) {
          if (i != myNodeId) {
              _db.removeNode(i);
          }
      }
      _settings.saveNodesSnapshot(_db);
      LOG_INFO("BLE", "Node database cleared via App command");
 }
 
 void DBManager::setAnchor(float lat, float lon) {
     _gps.setAnchorLocation(lat, lon);
     
     _packer.updateLonScale(lat);
     _lastScaleLat = lat;
     _isLonScaleSet = true;
 
     if (!_gps.isHardwarePresent()) {
         _settings.saveStaticCoordinates(lat, lon);
         LOG_INFO("SYS", "Device has no GPS hardware. Static anchor persisted to NVS.");
     }
 }