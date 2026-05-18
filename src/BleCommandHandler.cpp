/**
 * Project: Naviga-Dongle
 * File: BleCommandHandler.cpp
 * Version: 1.46.7
 * Description: Реализация диспетчера команд Bluetooth.
 * Изменение: Добавлена обработка флага _ble.hasNewAnchor для вызова _db.setAnchor.
 */

 #include "BleCommandHandler.h"

 BleCommandHandler::BleCommandHandler(BleManager& ble, DBManager& db, SettingsManager& settings,
                                      TxManager& tx, NodeDatabase& nodeDB, 
                                      const uint8_t& myNodeId, uint8_t& myNodeType)
     : _ble(ble), _db(db), _settings(settings), _tx(tx), _nodeDB(nodeDB), 
       _myNodeId(myNodeId), _myNodeType(myNodeType) 
 {
     _syncBookmark = 0; 
     _lastSyncSendTime = 0; 
 }
 
 void BleCommandHandler::process() {
     // 1. Запрос на отправку локальных Имени/Роли Оператору
     if (_ble.requestIdentitySync) {
         _ble.requestIdentitySync = false;
         _ble.sendIdentity(_settings.settings.nodeId,
                           _settings.settings.nodeName,
                           _settings.settings.nodeType);
         LOG_INFO("BLE", "Sent Identity config to App");
     }
 
     // 2. Запрос на отправку системных таймингов Оператору
     if (_ble.requestSysConfigSync) {
         _ble.requestSysConfigSync = false;
         _ble.sendSysConfig(_settings.settings.txIntervalMoving,
                            _settings.settings.txIntervalStill,
                            _settings.settings.nodeConnectionTimeout,
                            _settings.settings.nodeActiveTimeoutMs);
         LOG_INFO("BLE", "Sent System Config to App");
     }
 
     // 3. Запрос на выгрузку всей топологии сети (СТАРТ АВТОМАТА)
     if (_ble.requestFullSync) {
         _ble.requestFullSync = false;
         _syncBookmark = 1; 
         LOG_INFO("BLE", "Started Async Full Topology Sync...");
     }
 
     // 4. Запрос на очистку базы соседей
     if (_ble.requestClearDB) {
         _ble.requestClearDB = false;
         _db.clearDatabase(_myNodeId);
     }
 
     // 5. Оператор прислал новые настройки Идентификации (Имя/Роль)
     if (_ble.hasNewIdentity) {
         _ble.hasNewIdentity = false;
 
         _myNodeType = _ble.newIdentity.myRole;
         _settings.settings.nodeType = _myNodeType;
 
         strncpy(_settings.settings.nodeName, _ble.newIdentity.myName, sizeof(_settings.settings.nodeName) - 1);
         _settings.settings.nodeName[sizeof(_settings.settings.nodeName) - 1] = '\0';
         _settings.save();
 
         _tx.sendNodeInfo(_settings.settings.nodeName, _myNodeType, TX_CRITICAL);
         _nodeDB.updateNodeInfo(_myNodeId, _settings.settings.nodeName, _myNodeType);
         _settings.saveNodesSnapshot(_nodeDB);
         
         LOG_INFO("BLE", "Identity updated from App (ID %d remained unchanged)", _myNodeId);
     }
 
     // 6. Оператор прислал новые системные тайминги
     if (_ble.hasNewSysConfig) {
         _ble.hasNewSysConfig = false;
         _settings.settings.txIntervalMoving = _ble.newSysConfig.txIntervalMoving;
         _settings.settings.txIntervalStill = _ble.newSysConfig.txIntervalStill;
         _settings.settings.nodeConnectionTimeout = _ble.newSysConfig.nodeConnectionTimeout;
         _settings.settings.nodeActiveTimeoutMs = _ble.newSysConfig.nodeActiveTimeoutMs;
         _settings.save();
         LOG_INFO("BLE", "SysConfig updated from App and saved");
     }
 
     // 7. Оператор прислал команду на жесткий сброс (Factory Reset)
     if (_ble.requestReset) {
         LOG_INFO("BLE", "Executing FACTORY RESET via App Command...");
         _settings.factoryReset();
         delay(500);
         ESP.restart();
     }
 
     // НОВОЕ 1.46.7: Прием и обработка опорной точки от смартфона
     if (_ble.hasNewAnchor) {
         _ble.hasNewAnchor = false;
         _db.setAnchor(_ble.newAnchorLat, _ble.newAnchorLon);
         LOG_INFO("BLE", "Anchor coordinates processed via App: Lat=%.6f, Lon=%.6f", _ble.newAnchorLat, _ble.newAnchorLon);
     }
 
     // ====================================================================
     // 8. АСИНХРОННЫЙ АВТОМАТ ВЫГРУЗКИ БАЗЫ (State Machine)
     // ====================================================================
     if (_syncBookmark > 0) {
         
         if (millis() - _lastSyncSendTime < BLE_SYNC_PACKET_DELAY_MS) {
             return; 
         }
         
         while (_syncBookmark < 255) {
             uint8_t idToCheck = _syncBookmark;
             _syncBookmark++; 
 
             const NodeRecord *node = _nodeDB.getNode(idToCheck);
             if (node != nullptr && node->isActive) {
                 BleEvtNodeUpdate update;
                 update.opCode = EVT_NODE_UPDATE;
                 update.nodeId = node->nodeId;
                 update.nodeRole = node->type;
                 strncpy(update.nodeName, node->nodeName, sizeof(update.nodeName) - 1);
                 update.nodeName[sizeof(update.nodeName) - 1] = '\0';
                 update.lat = node->lat;
                 update.lon = node->lon;
                 update.snr = node->snr;
                 update.lastSeenAge = millis() - node->lastSeen;
 
                 _ble.sendNodeUpdate(update);
                 _lastSyncSendTime = millis();
                 return; 
             }
         }
 
         _syncBookmark = 0; 
         LOG_INFO("BLE", "Async Full Topology Sync Completed!");
     }
 }