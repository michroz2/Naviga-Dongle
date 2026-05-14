/**
 * Project: Naviga-Dongle
 * File: RxManager.cpp
 * Version: 1.46.5
 * Description: Реализация диспетчера входящего эфира.
 * Изменение: Внедрена дельта-оптимизация BLE-трафика (пакеты 0x15 и 0x16) 
 * согласно Контракту 1.46.4.
 */

 #include "RxManager.h"

 // Определяем глобальный аппаратный флаг
 volatile bool receivedFlag = false;
 
 // Реализация ISR
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 void setFlag(void) {
     receivedFlag = true;
 }
 
 RxManager::RxManager(RadioManager& radio, Retranslation& router, PacketManager& packetManager,
                      NodeDatabase& nodeDB, TxManager& txManager, BleManager& bleManager,
                      GpsManager& gps, SettingsManager& settingsManager,
                      const uint8_t& myNodeId, const uint8_t& myNodeType, const uint8_t& myMsgSeq,
                      uint32_t& lastHeartbeatTime, CollisionCallback collisionCb)
     : _radio(radio), _router(router), _packetManager(packetManager),
       _nodeDB(nodeDB), _txManager(txManager), _bleManager(bleManager),
       _gps(gps), _settingsManager(settingsManager),
       _myNodeId(myNodeId), _myNodeType(myNodeType), _myMsgSeq(myMsgSeq),
       _lastHeartbeatTime(lastHeartbeatTime), _collisionCb(collisionCb)
 {
 }
 
 bool RxManager::hasNewPacket() const {
     return receivedFlag;
 }
 
 // Внутренняя логика расчета джиттера перенесена сюда из main.cpp
 uint32_t RxManager::calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr) {
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
         if (maxMs < minMs)
             maxMs = minMs;
     }
 
     // Мапим задержку на основании качества сигнала (SNR)
     // Чем лучше сигнал, тем БОЛЬШЕ задержка (передает дальний узел)
     long snrInt = constrain((long)snr, -15, 5);
     uint32_t baseDelay = map(snrInt, -15, 5, minMs, maxMs);
     
     return baseDelay +
            random(0,
                  50); // Добавляем небольшую случайность для разрешения коллизий
 }
 
 void RxManager::process() {
     // Внимание: проверка receivedFlag дублируется (обычно мы заходим сюда если hasNewPacket == true),
     // но это защита на случай прямого вызова
     if (receivedFlag) {
         noInterrupts();
         receivedFlag = false;
         interrupts(); // Сброс ISR флага
         
         size_t len = _radio.getPacketLength();
         if (len > 0) {
             uint8_t rxBuffer[256];             
             int state = _radio.readData(rxBuffer, len); 
             if (state == RADIOLIB_ERR_NONE) {
                 
                 float currentSNR = _radio.getSNR();
                 
                 // Если пакет прошел базовую проверку на размер
                 if (len >= sizeof(NavigaHeader)) {
                     NavigaHeader rxHeader;
                     memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader)); // Извлекаем заголовок
                     size_t payloadLen = len - sizeof(NavigaHeader);
                     
                     bool isCollision = false;
                     bool isOwnEcho = false;
 
                     // Проверка на коллизии (конфликт ID)
                     if (rxHeader.relayId == _myNodeId) {
                         isCollision = true; // Кто-то ретранслирует под нашим ID
                     } else if (rxHeader.senderId == _myNodeId) {
                         // Кто-то отправляет оригинальный пакет с нашим ID. Нужно проверить,
                         // не наше ли это эхо.
                         int8_t seqDiff = (int8_t)(_myMsgSeq - rxHeader.msgSeq);
                         if (seqDiff <= 0 || seqDiff > 10) {
                             isCollision = true; // Коллизия!
                         } else {
                             isOwnEcho =
                                 true; // Наше собственное эхо, перехваченное от ретранслятора
                         } 
                     } 
 
                     if (isCollision) {
                         if (_collisionCb) _collisionCb();
                     } 
 
                     // Обновляем показатель SNR для узла, который физически передал нам
                     // пакет (relayId)
                     if (rxHeader.relayId != _myNodeId) {
                         _nodeDB.updateNodeSNR(rxHeader.relayId, currentSNR);
                     } 
 
                     if (isOwnEcho) {
                         // Эхо игнорируем
                     } else if (!_router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d",
                                    rxHeader.getType(), payloadLen);
                     } else if (_router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         // --- IMPLICIT ACK (Подавление перехватом) ---
                         // Если мы услышали пакет, который уже знаем (дубликат),
                         // мы проверяем векторный фильтр. Если мы тупик, мы отменяем свою
                         // задачу ретрансляции
                         if (!_nodeDB.hasNodesInOppositeDirection(rxHeader.relayId)) {
                             _txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                         } 
                     } else {
                         // --- ПАКЕТ УСПЕШНО ПРОШЕЛ ВСЕ ФИЛЬТРЫ ---
                         LOG_INFO("LORA",
                                    "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, SNR: "
                                    "%.1f)",
                                    rxHeader.getType(), rxHeader.senderId, rxHeader.relayId,
                                    rxHeader.msgSeq, currentSNR);
                         
                         // Флаг для определения, впервые ли мы слышим этот узел
                         bool isNewNode = !_nodeDB.isNodeActive(rxHeader.senderId);
                         
                         // Распаковка payload в зависимости от типа
                         _packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen);
 
                         // --- ДЕЛЬТА-ОПТИМИЗАЦИЯ BLE (v1.46.5) ---
                         // Уведомление смартфона о новых данных узла по BLE.
                         // Используем легкие пакеты 0x15/0x16 для экономии трафика.
                         if (_bleManager.getBleStatus() == BLE_CONNECTED) {
                             const NodeRecord* node = _nodeDB.getNode(rxHeader.senderId);
                             if (node != nullptr) {
                                 if (rxHeader.getType() == MSG_COORDS) {
                                     // Шлем дельта-координаты (14 байт)
                                     _bleManager.sendNodeCoords(node->nodeId, node->lat, node->lon, currentSNR);
                                 } else if (rxHeader.getType() == MSG_NODE_INFO) {
                                     // Шлем дельта-инфо (27 байт)
                                     _bleManager.sendNodeInfoUpdate(node->nodeId, node->type, node->nodeName);
                                 }
                             }
                         }
 
                         // Вежливость: Если это новый узел, планируем ответное приветствие
                         // (NodeInfo), чтобы он узнал о нашем существовании
                         if (isNewNode && rxHeader.senderId != _myNodeId) {
                             uint32_t currentMillis = millis();
                             uint32_t jitterMs = random(MIN_GREETING_NODEINFO_JITTER,
                                                        MAX_GREETING_NODEINFO_JITTER);
                             
                             // Изменяем таймер через переданную ссылку
                             _lastHeartbeatTime = currentMillis - HEARTBEAT_INTERVAL_MS + jitterMs;
                             
                             LOG_INFO("SYS",
                                        "New Node %d discovered! NodeInfo reply scheduled",
                                        rxHeader.senderId);
                             
                             // Сохраняем обновленную базу в NVS
                             _settingsManager.saveNodesSnapshot(_nodeDB);
                         } 
 
                         // --- ПРИНЯТИЕ РЕШЕНИЯ О РЕТРАНСЛЯЦИИ ---
                         // Запрашиваем скорость аппарата у глобального GPS для Таможни
                         float currentSpeed = _gps.getSpeed();
                         
                         if (_router.shouldRetransmit(rxHeader, _nodeDB, _myNodeType, currentSpeed)) {
                             uint8_t senderRole = NODE_STALKER; 
                             const NodeRecord* senderNode = _nodeDB.getNode(rxHeader.senderId);
                             if (senderNode != nullptr) {
                                 senderRole = senderNode->type;
                             }
                             
                             // Вычисляем Adaptive Jitter и ставим в очередь TxManager
                             uint32_t calculatedJitter = calculateRelayJitter(_myNodeType, senderRole, currentSNR);
                             _txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader),
                                                  payloadLen, calculatedJitter);
                         } 
                     } 
                 } 
             } 
         } 
         _radio.startReceive(); // Возвращаем радио в режим приема
     } 
 }