/**
 * File: TxManager.cpp
 * Version: 1.1.0
 */
 #include "TxManager.h"
 #include "logger.h"
 
 extern volatile bool receivedFlag;
 
 TxManager::TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq)
     : _radio(radio), _packer(packer), _myNodeId(nodeId), _myMsgSeq(msgSeq) {
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         _queue[i].isActive = false;
     }
     _activeJobIndex = -1;
     _jitterStartTime = 0;
     _jitterDelay = 0;
 }
 
 bool TxManager::enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs) {
     if (payloadLen > MAX_PAYLOAD_SIZE) return false;
 
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (!_queue[i].isActive) {
             _queue[i].header = header;
             if (payloadLen > 0 && payload != nullptr) {
                 memcpy(_queue[i].payload, payload, payloadLen);
             }
             _queue[i].payloadLen = payloadLen;
             _queue[i].priority = priority;
             _queue[i].readyTime = millis() + delayMs;
             _queue[i].isActive = true;
             return true;
         }
     }
     LOG_WARN("TX", "TxQueue is FULL! Dropping packet.");
     return false;
 }
 
 void TxManager::sendNodeInfo(const char* nodeName, TxPriority priority) {
     NavigaHeader txHeader;
     txHeader.senderId = _myNodeId;
     txHeader.relayId = _myNodeId;
     txHeader.msgSeq = _myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_NODE_INFO, 15);
 
     PayloadNodeInfo infoPayload;
     strncpy(infoPayload.nodeName, nodeName, sizeof(infoPayload.nodeName) - 1);
     infoPayload.nodeName[sizeof(infoPayload.nodeName) - 1] = '\0';
 
     enqueue(txHeader, (const uint8_t*)&infoPayload, sizeof(PayloadNodeInfo), priority, 0);
 }
 
 void TxManager::sendCoords(float lat, float lon, TxPriority priority) {
     NavigaHeader txHeader;
     txHeader.senderId = _myNodeId;
     txHeader.relayId = _myNodeId;
     txHeader.msgSeq = _myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_COORDS, 15);
 
     uint32_t packedCoords = _packer.pack(lat, lon);
     enqueue(txHeader, (const uint8_t*)&packedCoords, sizeof(uint32_t), priority, 0);
 }
 
 bool TxManager::enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, float snr) {
     long snrConstrained = (long)snr;
     if (snrConstrained < -11) snrConstrained = -11;
     if (snrConstrained > 5) snrConstrained = 5;
 
     // Geographic Routing: Ближе к границе приема (-11) -> 100 мс. Близко (+5) -> 1000 мс.
     uint32_t baseDelay = map(snrConstrained, -11, 5, 100, 1000);
 
     NavigaHeader relayHeader = header;
     relayHeader.setTypeAndTTL(static_cast<NavigaMessageType>(header.getType()), header.getTTL() - 1);
     relayHeader.relayId = _myNodeId;
 
     LOG_INFO("QUEUE", "Pkt Seq %d (Relay) queued. SNR: %.1f -> Base Delay: %d ms", header.msgSeq, snr, baseDelay);
     return enqueue(relayHeader, payload, payloadLen, TX_RELAY, baseDelay);
 }
 
 void TxManager::abortRelay(uint8_t senderId, uint8_t msgSeq) {
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (_queue[i].isActive && _queue[i].header.senderId == senderId && _queue[i].header.msgSeq == msgSeq) {
             _queue[i].isActive = false;
             // Если пакет готовился к отправке прямо сейчас, прерываем процесс
             if (_activeJobIndex == i) {
                 _activeJobIndex = -1;
             }
             LOG_INFO("QUEUE", "Relay ABORTED for Seq %d (Suppressed by network)", msgSeq);
             return;
         }
     }
 }
 
 void TxManager::processQueue() {
     // Правило №1: Если эфир занят, мы мгновенно прерываем подготовку и замираем.
     if (receivedFlag) {
         _activeJobIndex = -1;
         return;
     }
 
     uint32_t now = millis();
 
     // Правило №2: Если конвейер пуст, ищем самого достойного кандидата
     if (_activeJobIndex == -1) {
         int8_t bestIndex = -1;
         TxPriority bestPriority = TX_RELAY; // Начинаем поиск с самого низкого приоритета
 
         for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
             if (_queue[i].isActive && now >= _queue[i].readyTime) {
                 // Ищем пакет с наивысшим приоритетом (численно меньшим)
                 if (bestIndex == -1 || _queue[i].priority < bestPriority) {
                     bestIndex = i;
                     bestPriority = _queue[i].priority;
                 }
             }
         }
 
         // Кандидат найден! Назначаем ему Jitter и запускаем таймер.
         if (bestIndex != -1) {
             _activeJobIndex = bestIndex;
             _jitterStartTime = now;
             
             switch(bestPriority) {
                 case TX_CRITICAL: _jitterDelay = random(20, 50); break;
                 case TX_HIGH:     _jitterDelay = random(50, 150); break;
                 case TX_NORMAL:   _jitterDelay = random(100, 300); break;
                 case TX_RELAY:    _jitterDelay = random(50, 200); break; // Добавка к базовой SNR-задержке
             }
         }
     }
 
     // Правило №3: Если Jitter дотикал, а эфир всё ещё свободен — стреляем!
     if (_activeJobIndex != -1) {
         if (now - _jitterStartTime >= _jitterDelay) {
             
             uint8_t txBuffer[sizeof(NavigaHeader) + MAX_PAYLOAD_SIZE];
             size_t totalLen = sizeof(NavigaHeader) + _queue[_activeJobIndex].payloadLen;
             
             memcpy(txBuffer, &_queue[_activeJobIndex].header, sizeof(NavigaHeader));
             if (_queue[_activeJobIndex].payloadLen > 0) {
                 memcpy(txBuffer + sizeof(NavigaHeader), _queue[_activeJobIndex].payload, _queue[_activeJobIndex].payloadLen);
             }
 
             LOG_INFO("TX", "Transmitting Type %d, Seq %d. Priority: %d", 
                      _queue[_activeJobIndex].header.getType(), 
                      _queue[_activeJobIndex].header.msgSeq, 
                      _queue[_activeJobIndex].priority);
 
             _radio.standby();
             _radio.transmit(txBuffer, totalLen);
             receivedFlag = false;
             _radio.startReceive();
 
             _queue[_activeJobIndex].isActive = false; 
             _activeJobIndex = -1; 
         }
     }
 }