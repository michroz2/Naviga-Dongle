/**
 * File: TxManager.cpp
 * Version: 1.19 Изменение: Делегирование расчета джиттера внешнему коду при ретрансляции (Шаг 2).
 * Description: Реализация TxManager (MAC-диспетчер и управление очередями отправки).
 */
 #include "TxManager.h"
 #include "logger.h"
 
 extern volatile bool receivedFlag; // Флаг прерывания из main.cpp для определения занятости эфира
 
 TxManager::TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq)
     : _radio(radio), _packer(packer), _myNodeId(nodeId), _myMsgSeq(msgSeq) {
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         _queue[i].isActive = false;
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
     _activeJobIndex = -1;
     _jitterStartTime = 0;
     _jitterDelay = 0;
 } // TxManager::TxManager()
 
 // Базовый метод постановки задачи в очередь (CSMA/CA)
 bool TxManager::enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs) {
     if (payloadLen > MAX_PAYLOAD_SIZE) return false;
 
     // Ищем свободный слот в очереди
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (!_queue[i].isActive) {
             _queue[i].header = header;
             if (payloadLen > 0 && payload != nullptr) {
                 memcpy(_queue[i].payload, payload, payloadLen);
             } // if (payloadLen > 0 && ...)
             _queue[i].payloadLen = payloadLen;
             _queue[i].priority = priority;
             _queue[i].readyTime = millis() + delayMs; // Время, когда пакет можно начать обрабатывать
             _queue[i].isActive = true;
             return true;
         } // if (!_queue[i].isActive)
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
     
     LOG_WARN("TX", "TxQueue is FULL! Dropping packet.");
     return false;
 } // TxManager::enqueue()
 
 // Упаковка и постановка в очередь информации об узле (Heartbeat / Collision reset)
 void TxManager::sendNodeInfo(const char* nodeName, uint8_t nodeType, TxPriority priority) {
     NavigaHeader txHeader;
     txHeader.senderId = _myNodeId;
     txHeader.relayId = _myNodeId; // Изначально мы являемся и отправителем и ретранслятором
     txHeader.msgSeq = _myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_NODE_INFO, DEFAULT_TTL);
 
     PayloadNodeInfo infoPayload;
     infoPayload.nodeType = nodeType; 
     strncpy(infoPayload.nodeName, nodeName, sizeof(infoPayload.nodeName) - 1);
     infoPayload.nodeName[sizeof(infoPayload.nodeName) - 1] = '\0';
 
     enqueue(txHeader, (const uint8_t*)&infoPayload, sizeof(PayloadNodeInfo), priority, 0);
     LOG_INFO("TX", "Enqueued NODE_INFO: Name=%s, Type=%d", nodeName, nodeType);
 } // TxManager::sendNodeInfo()
 
 // Упаковка и постановка в очередь координат
 void TxManager::sendCoords(float lat, float lon, TxPriority priority) {
     NavigaHeader txHeader;
     txHeader.senderId = _myNodeId;
     txHeader.relayId = _myNodeId;
     txHeader.msgSeq = _myMsgSeq++;
     txHeader.setTypeAndTTL(MSG_COORDS, DEFAULT_TTL);
 
     // Сжатие координат перед постановкой
     uint32_t packedCoords = _packer.pack(lat, lon);
     enqueue(txHeader, (const uint8_t*)&packedCoords, sizeof(uint32_t), priority, 0);
 } // TxManager::sendCoords()
 
 // Постановка в очередь чужого пакета (Ретрансляция) с заданным окном задержки (Джиттером)
 // ИЗМЕНЕНИЕ 1.19: Принимаем готовый delayMs вместо snr
 bool TxManager::enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, uint32_t delayMs) {
     if (payloadLen > MAX_PAYLOAD_SIZE) return false;
 
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (!_queue[i].isActive) {
             _queue[i].header = header;
             
             // Декрементируем TTL (Time-to-Live)
             uint8_t currentTTL = header.getTTL();
             uint8_t newTTL = (currentTTL > 0) ? (currentTTL - 1) : 0;
             _queue[i].header.setTypeAndTTL(header.getType(), newTTL);
             
             // Записываем СЕБЯ как последнего ретранслятора
             _queue[i].header.relayId = _myNodeId;
 
             if (payloadLen > 0 && payload != nullptr) {
                 memcpy(_queue[i].payload, payload, payloadLen);
             } // if (payloadLen > 0 && payload != nullptr)
             
             _queue[i].payloadLen = payloadLen;
             _queue[i].priority = TX_RELAY; // Приоритет ретрансляции
             
             // Задержка (Adaptive Jitter) теперь сразу прибавляется к времени готовности задачи
             _queue[i].readyTime = millis() + delayMs;
             
             _queue[i].isActive = true;
             return true;
         } // if (!_queue[i].isActive)
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
     return false;
 } // TxManager::enqueueRelay()
 
 // Implicit ACK (Подавление перехватом). Вызывается из main, если в эфире услышан дубликат.
 void TxManager::abortRelay(uint8_t senderId, uint8_t msgSeq) {
     for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
         if (_queue[i].isActive && _queue[i].header.senderId == senderId && _queue[i].header.msgSeq == msgSeq) {
             _queue[i].isActive = false; // Удаляем задачу из очереди
             if (_activeJobIndex == i) {
                 _activeJobIndex = -1;   // Если она уже была активна на отправку, сбрасываем состояние
             } // if (_activeJobIndex == i)
             LOG_INFO("QUEUE", "Relay ABORTED for Seq %d (Suppressed by network)", msgSeq);
             return;
         } // if (_queue[i].isActive && ...)
     } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
 } // TxManager::abortRelay()
 
 // Диспетчер отправки. Вызывается регулярно в loop(). Реализует механизм CSMA/CA.
 void TxManager::processQueue() {
     // Защита от коллизий: если радиомодуль ловит шум (принимает пакет), 
     // мы сбрасываем попытку отправки и ждем освобождения эфира.
     if (receivedFlag) {
         _activeJobIndex = -1;
         return;
     } // if (receivedFlag)
 
     uint32_t now = millis();
 
     // Если нет активной задачи в процессе, выбираем самую приоритетную
     if (_activeJobIndex == -1) {
         int8_t bestIndex = -1;
         TxPriority bestPriority = TX_RELAY; 
 
         // Сканируем очередь на наличие готовых к отправке пакетов (с учетом задержки readyTime)
         for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++) {
             if (_queue[i].isActive && now >= _queue[i].readyTime) {
                 if (bestIndex == -1 || _queue[i].priority < bestPriority) {
                     bestIndex = i;
                     bestPriority = _queue[i].priority;
                 } // if (bestIndex == -1 || ...)
             } // if (_queue[i].isActive && now >= _queue[i].readyTime)
         } // for (uint8_t i = 0; i < TX_QUEUE_SIZE; i++)
 
         // Нашли лучшую задачу. Присваиваем ей финальный системный джиттер перед отправкой.
         if (bestIndex != -1) {
             _activeJobIndex = bestIndex;
             _jitterStartTime = now;
             
             // Добавляем микро-задержки для предотвращения одновременного выхода в эфир 
             // нескольких устройств после окончания приема чужого пакета
             switch(bestPriority) {
                 case TX_CRITICAL: _jitterDelay = random(10, 50); break;
                 case TX_HIGH:     _jitterDelay = random(50, 150); break;
                 case TX_NORMAL:   _jitterDelay = random(100, 300); break;
                 case TX_RELAY: {
                     // ИЗМЕНЕНИЕ 1.19: Джиттер уже был учтен в readyTime при помещении в очередь.
                     // Ставим 0, чтобы передача сработала немедленно при выборе задачи.
                     _jitterDelay = 0; 
                     break;
                 }
             } // switch(bestPriority)
         } // if (bestIndex != -1)
     } // if (_activeJobIndex == -1)
 
     // Если активная задача есть и таймер джиттера истек — выходим в эфир
     if (_activeJobIndex != -1) {
         if (now - _jitterStartTime >= _jitterDelay) {
             
             // Подготавливаем буфер для физической передачи
             uint8_t txBuffer[sizeof(NavigaHeader) + MAX_PAYLOAD_SIZE];
             size_t totalLen = sizeof(NavigaHeader) + _queue[_activeJobIndex].payloadLen;
             
             memcpy(txBuffer, &_queue[_activeJobIndex].header, sizeof(NavigaHeader));
             if (_queue[_activeJobIndex].payloadLen > 0) {
                 memcpy(txBuffer + sizeof(NavigaHeader), _queue[_activeJobIndex].payload, _queue[_activeJobIndex].payloadLen);
             } // if (_queue[_activeJobIndex].payloadLen > 0)
 
             LOG_INFO("TX", "Transmitting Type %d, Seq %d. Priority: %d", 
                      _queue[_activeJobIndex].header.getType(), 
                      _queue[_activeJobIndex].header.msgSeq, 
                      _queue[_activeJobIndex].priority);
 
             _radio.standby(); // Переводим радио в спящий режим перед передачей
             _radio.transmit(txBuffer, totalLen); // Блокирующая передача!
             receivedFlag = false; // Очищаем флаг приема, который мог подняться из-за наводок при передаче
             _radio.startReceive(); // Возвращаем в режим приема
 
             // Задача выполнена, освобождаем слот
             _queue[_activeJobIndex].isActive = false; 
             _activeJobIndex = -1; 
         } // if (now - _jitterStartTime >= _jitterDelay)
     } // if (_activeJobIndex != -1)
 } // TxManager::processQueue()