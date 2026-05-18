/**
 * Project: Naviga-Dongle
 * File: SmartPositionManager.cpp
 * Version: 1.46.7
 * Description: Реализация логики адаптивной отправки координат.
 * Изменение: Использование метода gps.hasFix() для жесткого запрета ложной отправки привязки в эфир.
 */

 #include "SmartPositionManager.h"

 SmartPositionManager::SmartPositionManager(GpsManager& _gps, NodeDatabase& _nodeDB, TxManager& _txManager, 
                                            BleManager& _bleManager, SettingsManager& _settingsManager,
                                            const uint8_t& _myNodeId, const uint8_t& _myNodeType)
     : gps(_gps), nodeDB(_nodeDB), txManager(_txManager), 
       bleManager(_bleManager), settingsManager(_settingsManager),
       myNodeId(_myNodeId), myNodeType(_myNodeType), lastTxTime(0) 
 {
 }
 
 void SmartPositionManager::resetTimer() {
     lastTxTime = millis();
 }
 
 void SmartPositionManager::process() {
     // ИЗМЕНЕНИЕ 1.46.7: Если нет спутникового фикса (а есть только RAM-опора),
     // мы категорически запрещаем слать наши координаты в LoRa эфир.
     if (!gps.hasFix()) {
         if (millis() - lastTxTime >= settingsManager.settings.txIntervalStill) {
             LOG_WARN("TX", "Skip SmartTX: No true satellite fix available.");
             lastTxTime = millis(); 
         }
         return;
     }
 
     uint32_t now = millis();
     float currentSpeed = gps.getSpeed();
     bool shouldTransmit = false;
     
     const NodeRecord* myRecord = nodeDB.getNode(myNodeId);
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
         txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
         
         nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0); 
         
         if (bleManager.getBleStatus() == BLE_CONNECTED) {
             bleManager.sendNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0.0f);
             LOG_INFO("BLE", "Sync: Local position (delta 0x15) sent to Smartphone");
         }
 
         lastTxTime = now;
     }
 }