/**
 * File: NetworkManager.cpp
 * Version: 1.00 (Refactored from main.cpp v1.43)
 * Description: Реализация сетевой логики. Весь код перенесен из main.cpp без изменений алгоритмов.
 */

 #include "NetworkManager.h"
 #include "logger.h"
 
 NetworkManager::NetworkManager(RadioManager& radio, NodeDatabase& db, DisplayManager& disp, 
                                GpsManager& gps, TxManager& tx, BleManager& ble, 
                                Retranslation& router, PacketManager& packetMgr)
     : _radio(radio), _nodeDB(db), _display(disp), _gps(gps), 
       _txManager(tx), _bleManager(ble), _router(router), _packetManager(packetMgr) {
 }
 
 // Расчет джиттера (рандомизированной задержки) для умной ретрансляции пакета
 uint32_t NetworkManager::calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr) {
     uint32_t minMs, maxMs;
     
     // Определяем базовые окна джиттера в зависимости от нашей роли
     if (myRole == NODE_RELAY) {
         minMs = RELAY_JITTER_MIN_MS;
         maxMs = RELAY_JITTER_MAX_MS;
     } else {
         minMs = STALKER_JITTER_MIN_MS;
         maxMs = STALKER_JITTER_MAX_MS;
     }
 
     // VIP-Маршрутизация: Если мы ретранслируем пакет ТРЕКЕРА, даем ему зеленый свет
     if (senderRole == NODE_TRACKER) {
         maxMs /= 2; // Ускоряем в 2 раза
         if (maxMs < minMs) maxMs = minMs; 
     }
 
     // Мапим задержку на основании качества сигнала (SNR)
     // Чем лучше сигнал, тем БОЛЬШЕ задержка (передает дальний узел)
     long snrInt = constrain((long)snr, -15, 5);
     uint32_t baseDelay = map(snrInt, -15, 5, minMs, maxMs);
     
     return baseDelay + random(0, 50); // Добавляем небольшую случайность для разрешения коллизий
 }
 
 // Расчет показателя "качества связи" для вывода на экран (от 1 до 10)
 int NetworkManager::getConnectionQuality(uint8_t targetId, uint8_t myNodeId) {
     if (targetId == myNodeId) return 10; 
 
     const NodeRecord* target = _nodeDB.getNode(targetId);
     if (target == nullptr || targetId == 0) return 0;
     
     // Если от узла давно не было вестей, качество 0
     if (millis() - target->lastSeen > settingsManager.settings.nodeConnectionTimeout) return 0;
     
     if (target->snr <= -99.0f) return 0; 
 
     // Мапим физический SNR в читаемый балл (1-10)
     int q = map((long)target->snr, -11, 5, 1, 10);
     if (q < 1) q = 1;
     if (q > 10) q = 10;
     return q;
 }
 
 // Обработка коллизии: если два узла заняли один ID
 void NetworkManager::handleCollision(uint8_t& myNodeId, uint8_t& myMsgSeq, uint8_t myNodeType) {
     uint8_t oldId = myNodeId;
     _nodeDB.removeNode(oldId); 
     
     // Генерируем новый уникальный ID
     randomSeed(esp_random());
     do {
         myNodeId = random(1, 255);
     } while (_nodeDB.getNode(myNodeId) != nullptr); // Проверяем, свободен ли он в базе
     
     _nodeDB.addNode(myNodeId); 
     
     LOG_WARN("COLLISION", "ID %d is taken! Switched to new ID: %d", oldId, myNodeId);
     
     myMsgSeq = 0; // Сбрасываем счетчик пакетов
     
     char myName[24]; // Буфер расширен до 24 байт
     snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
     
     // Рассылаем новый ID по сети с наивысшим приоритетом
     _txManager.sendNodeInfo(myName, myNodeType, TX_CRITICAL);
     _nodeDB.updateNodeInfo(myNodeId, myName, myNodeType); 
  
     // Сохраняем изменения в энергонезависимую память (NVS)
     settingsManager.settings.nodeId = myNodeId;
     settingsManager.save();
     settingsManager.saveNodesSnapshot(_nodeDB);
 }
 
 // Универсальная функция первоначального сканирования и немого периода (Warm/Cold Start)
 void NetworkManager::scanNetwork(bool isWarmStart, uint32_t networkScanDuration, uint8_t& myNodeId) {
     if (isWarmStart) {
         LOG_INFO("SYS", "Warm Start: Silent listening for %d ms...", networkScanDuration);
     } else {
         LOG_INFO("SYS", "Cold Start: Scanning for %d ms...", networkScanDuration);
     }
     
     _display.toggleLed();
     
     uint32_t scanStart = millis();
     uint32_t lastDispUpdate = 0;
     
     // Мы не можем напрямую сбросить флаг прерывания, но можем подготовить радио
     _radio.startReceive(); 
     
     // Цикл немого прослушивания радиоэфира
     while (millis() - scanStart < networkScanDuration) {
         uint32_t now = millis();
         
         // Обновляем дисплей каждую секунду
         if (now - lastDispUpdate >= 1000) {
             lastDispUpdate = now;
             uint32_t left = (networkScanDuration - (now - scanStart)) / 1000;
             
             String startTitle = "Start-" + String(_bleManager.macSuffix);
             
             // Получаем общее количество узлов и вычитаем себя (Донгл)
             uint8_t totalNodes = _nodeDB.getActiveNodesCount();
             uint8_t foundNeighbors = (totalNodes > 0) ? (totalNodes - 1) : 0;
             
             _display.showStatus(startTitle, 
                                "Time left: " + String(left) + " s", 
                                "Neighbors: " + String(foundNeighbors), 
                                "Please wait...");
         } 
 
         // Обработка входящих пакетов во время сканирования
         // ВАЖНО: В режиме сканирования мы используем прямой опрос радио
         size_t len = _radio.getPacketLength();
         if (len >= sizeof(NavigaHeader)) {
             uint8_t rxBuffer[256];             
             int state = _radio.readData(rxBuffer, len); 
             if (state == RADIOLIB_ERR_NONE) {
                 NavigaHeader rxHeader;
                 memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                 size_t payloadLen = len - sizeof(NavigaHeader);
                 
                 if (_router.isValidPacket(rxHeader.getType(), payloadLen)) {
                     if (!_router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         _packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen);
                     } 
                 } 
             } 
             _radio.startReceive();
         } 
         
         _gps.update(); // Поддерживаем опрос GPS
     } 
     
     _display.toggleLed();
     
     // Если это Cold Start (или у нас почему-то нет ID), генерируем новый
     if (!isWarmStart || myNodeId == 0) {
         randomSeed(esp_random());
         do {
             myNodeId = random(1, 255);
         } while (_nodeDB.getNode(myNodeId) != nullptr); 
         
         _nodeDB.addNode(myNodeId); 
         LOG_INFO("SYS", "Scan complete. Selected unique Node ID: %d", myNodeId);
     } else {
         LOG_INFO("SYS", "Silent listening complete. Kept Node ID: %d", myNodeId);
     }
     
     _display.showStatus("Scan Complete", "My ID:", String(myNodeId), "Starting...");
     delay(2000);
 }  //NetworkManager.cpp