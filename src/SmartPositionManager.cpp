/**
 * Project: Naviga-Dongle
 * File: SmartPositionManager.cpp
 * Version: 1.00
 * Description: Реализация логики адаптивной отправки координат.
 */

 #include "SmartPositionManager.h"

 SmartPositionManager::SmartPositionManager(GpsManager& _gps, NodeDatabase& _nodeDB, TxManager& _txManager, 
                                            BleManager& _bleManager, SettingsManager& _settingsManager,
                                            const uint8_t& _myNodeId, const uint8_t& _myNodeType)
     : gps(_gps), nodeDB(_nodeDB), txManager(_txManager), 
       bleManager(_bleManager), settingsManager(_settingsManager),
       myNodeId(_myNodeId), myNodeType(_myNodeType), lastTxTime(0) 
 {
     // Конструктор инициализирует связи с объектами оркестратора
 }
 
 void SmartPositionManager::resetTimer() {
     lastTxTime = millis();
 }
 
 void SmartPositionManager::process() {
     // Если нет фикса GPS, работаем в режиме ожидания
     if (!gps.isValid()) {
         if (millis() - lastTxTime >= settingsManager.settings.txIntervalStill) {
             LOG_WARN("TX", "Skip TX: GPS location not valid.");
             lastTxTime = millis(); 
         }
         return;
     }
 
     uint32_t now = millis();
     float currentSpeed = gps.getSpeed();
     bool shouldTransmit = false;
     
     // Получаем запись о самом себе из базы данных для расчета дистанции
     const NodeRecord* myRecord = nodeDB.getNode(myNodeId);
     
     // Дистанция, пройденная с момента последней передачи координат в сеть
     float distFromLastTx = (myRecord != nullptr) ? myRecord->distance : 0.0f;
 
     // --- ЛОГИКА ПРИНЯТИЯ РЕШЕНИЯ (АЛГОРИТМ SMART TX) ---
 
     // 1. Условие активного движения
     if (distFromLastTx > MIN_MOVEMENT_METERS && currentSpeed > MIN_SPEED_KMPH) {
         if (now - lastTxTime >= settingsManager.settings.txIntervalMoving) {
             shouldTransmit = true;
             LOG_INFO("ACTION", "SmartTX Trigger: Movement (Dist: %.1fm, Speed: %.1f)", distFromLastTx, currentSpeed);
         }
     } 
     // 2. Условие "Крадущийся" (значительное смещение при низкой скорости)
     else if (distFromLastTx > SNEAK_MOVEMENT_METERS) {
         if (now - lastTxTime >= settingsManager.settings.txIntervalMoving) {
             shouldTransmit = true;
             LOG_INFO("ACTION", "SmartTX Trigger: Sneaking (Dist: %.1fm)", distFromLastTx);
         }
     }
     // 3. Условие статического Heartbeat (таймер покоя)
     else if (now - lastTxTime >= settingsManager.settings.txIntervalStill) {
         shouldTransmit = true;
         LOG_INFO("ACTION", "SmartTX Trigger: Static Heartbeat");
     }
 
     // --- ВЫПОЛНЕНИЕ ОТПРАВКИ ---
     if (shouldTransmit) {
         // А. Отправка в радиоэфир LoRa
         txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
         
         // Б. Обновление собственной позиции в локальной базе (чтобы сбросить счетчик дистанции)
         nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0); 
         
         // В. Симметричная выгрузка данных в смартфон по Bluetooth (если подключен)
         if (bleManager.getBleStatus() == BLE_CONNECTED) {
             BleEvtNodeUpdate update;
             update.opCode = EVT_NODE_UPDATE;
             update.nodeId = myNodeId;
             update.nodeRole = myNodeType;
             strncpy(update.nodeName, settingsManager.settings.nodeName, sizeof(update.nodeName)-1);
             update.nodeName[sizeof(update.nodeName)-1] = '\0';
             update.lat = gps.getLat();
             update.lon = gps.getLon();
             update.snr = 0.0f; 
             update.lastSeenAge = 0;
             
             bleManager.sendNodeUpdate(update);
             LOG_INFO("BLE", "Sync: Local position sent to Smartphone");
         }
 
         lastTxTime = now;
     }
 } //SmartPositionManager.cpp