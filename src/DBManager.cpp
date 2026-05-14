/**
 * Project: Naviga-Dongle
 * File: DBManager.cpp
 * Version: 1.43.8
 * Изменение: Асинхронная неблокирующая выгрузка базы (State Machine).
 * Description: Реализация логики контроллера базы данных узлов.
 */

 #include "DBManager.h"

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
     _db.removeNode(oldId);
 
     // Генерируем новый уникальный ID и обновляем ссылку в main.cpp
     myNodeId = generateUniqueId();
     _db.addNode(myNodeId);
 
     LOG_WARN("COLLISION", "ID %d is taken! Switched to new ID: %d", oldId, myNodeId);
 
     myMsgSeq = 0; 
 
     char myName[24]; 
     snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
 
     // Рассылаем новый ID по сети с наивысшим приоритетом
     _tx.sendNodeInfo(myName, myNodeType, TX_CRITICAL);
     _db.updateNodeInfo(myNodeId, myName, myNodeType);
 
     // Сохраняем изменения в энергонезависимую память
     _settings.settings.nodeId = myNodeId;
     _settings.save();
     _settings.saveNodesSnapshot(_db);
 }
 
 void DBManager::processBackgroundTasks(bool isFastTracker, uint8_t myNodeId) {
     uint32_t currentMillis = millis();
 
     if (_gps.isValid() && (currentMillis - _lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS)) {
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
 
 void DBManager::updateGeodata(bool isFastTracker) {
     if (_gps.isValid()) {
         float currentLat = _gps.getLat();
         if (!_isLonScaleSet || abs(currentLat - _lastScaleLat) > 1.0f) {
             _packer.updateLonScale(currentLat);
             _lastScaleLat = currentLat;
             _isLonScaleSet = true;
             LOG_INFO("SYS", "Longitude scale updated for Lat: %.4f", currentLat);
         }
 
         if (!isFastTracker) {
             for (int i = 1; i < 255; i++) {
                 const NodeRecord *node = _db.getNode(i);
                 if (node != nullptr && node->isActive) {
                     // Распаковка свежих координат
                     if (node->packedCoords != 0 && node->lat == 0.0f && node->lon == 0.0f) {
                         float unpLat, unpLon;
                         _packer.unpack(node->packedCoords, _gps.getLat(), _gps.getLon(), unpLat, unpLon);
                         _db.updateNodeCoords(i, unpLat, unpLon, node->packedCoords, false);
                     }
 
                     // Пересчет дистанции и азимута для UI и Векторного фильтра
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