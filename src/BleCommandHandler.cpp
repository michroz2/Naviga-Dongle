/**
 * Project: Naviga-Dongle
 * File: BleCommandHandler.cpp
 * Version: 1.43.7
 * Description: Реализация диспетчера команд Bluetooth.
 */

 #include "BleCommandHandler.h"

 BleCommandHandler::BleCommandHandler(BleManager& ble, DBManager& db, SettingsManager& settings,
                                      TxManager& tx, NodeDatabase& nodeDB, 
                                      const uint8_t& myNodeId, uint8_t& myNodeType)
     : _ble(ble), _db(db), _settings(settings), _tx(tx), _nodeDB(nodeDB), 
       _myNodeId(myNodeId), _myNodeType(myNodeType) 
 {
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
 
     // 3. Запрос на выгрузку всей топологии сети
     if (_ble.requestFullSync) {
         _ble.requestFullSync = false;
         _db.syncTopologyToBle(); // TODO: В будущем будет переведено на асинхронный вызов
     }
 
     // 4. Запрос на очистку базы соседей
     if (_ble.requestClearDB) {
         _ble.requestClearDB = false;
         _db.clearDatabase(_myNodeId);
     }
 
     // 5. Оператор прислал новые настройки Идентификации (Имя/Роль)
     if (_ble.hasNewIdentity) {
         _ble.hasNewIdentity = false;
 
         // ИСПРАВЛЕНИЕ АРХИТЕКТУРЫ: Мы ИГНОРИРУЕМ ID от Оператора (_ble.newIdentity.myNodeId).
         // Донгл сам управляет своим ID. Разрешаем менять только Роль и Имя.
         _myNodeType = _ble.newIdentity.myRole;
         _settings.settings.nodeType = _myNodeType;
 
         strncpy(_settings.settings.nodeName, _ble.newIdentity.myName, sizeof(_settings.settings.nodeName) - 1);
         _settings.settings.nodeName[sizeof(_settings.settings.nodeName) - 1] = '\0';
         _settings.save();
 
         // Оповещаем сеть и обновляем локальную базу
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
 } //BleCommandHandler.cpp