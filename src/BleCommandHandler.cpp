/**
 * Project: Naviga-Dongle
 * File: BleCommandHandler.cpp
 * Version: 1.46.6
 * Description: Реализация диспетчера команд Bluetooth.
 * Изменение: Асинхронная выгрузка базы: добавлен Rate-Limiting (20мс) и включен сам Донгл в выгрузку.
 */

 #include "BleCommandHandler.h"

 BleCommandHandler::BleCommandHandler(BleManager& ble, DBManager& db, SettingsManager& settings,
                                      TxManager& tx, NodeDatabase& nodeDB, 
                                      const uint8_t& myNodeId, uint8_t& myNodeType)
     : _ble(ble), _db(db), _settings(settings), _tx(tx), _nodeDB(nodeDB), 
       _myNodeId(myNodeId), _myNodeType(myNodeType) 
 {
     _syncBookmark = 0; // Изначально автомат выгрузки выключен
     _lastSyncSendTime = 0; // Инициализация таймера задержки
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
         _syncBookmark = 1; // Устанавливаем закладку на начало базы
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
 
     // ====================================================================
     // 8. АСИНХРОННЫЙ АВТОМАТ ВЫГРУЗКИ БАЗЫ (State Machine)
     // Размещен в конце, чтобы гарантированно отработать после других команд
     // ====================================================================
     if (_syncBookmark > 0) {
         
         // НОВОЕ 1.46.6: Rate-Limiting. Если с момента последней отправки
         // прошло менее 20 мс, возвращаем управление в основной цикл ESP32.
         if (millis() - _lastSyncSendTime < BLE_SYNC_PACKET_DELAY_MS) {
             return; 
         }
         
         // Сканируем базу, начиная с закладки
         while (_syncBookmark < 255) {
             uint8_t idToCheck = _syncBookmark;
             _syncBookmark++; // Сразу сдвигаем закладку для следующей итерации/цикла
 
             // ИЗМЕНЕНИЕ 1.46.6: Фильтр пропуска собственного узла удален.
             // Теперь при Full Sync мы выгружаем Оператору и свои полные данные (вкл. координаты).
 
             const NodeRecord *node = _nodeDB.getNode(idToCheck);
             if (node != nullptr && node->isActive) {
                 // Нашли активный узел (в том числе свой). Собираем и отправляем пакет.
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
                 
                 // Обновляем таймер после успешной выгрузки пакета в буфер BLE
                 _lastSyncSendTime = millis();
                 
                 // ВАЖНО: Прерываем выполнение функции. 
                 // Отправлена ровно 1 запись, процессор возвращается в loop().
                 return; 
             }
         }
 
         // Если цикл while завершился (закладка дошла до 255 и никого больше нет)
         _syncBookmark = 0; // Выключаем автомат
         LOG_INFO("BLE", "Async Full Topology Sync Completed!");
     }
 }