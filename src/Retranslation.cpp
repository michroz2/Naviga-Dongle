/**
 * File: Retranslation.cpp
 * Version: 1.2.7
 * Description: Реализация класса ретрансляции.
 * Добавлен расчет задержки ретрансляции в зависимости от SNR.
 */
 #include "Retranslation.h"
 #include "logger.h"
 
 Retranslation::Retranslation() {
     head = 0;
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         history[i].senderId = 0;
         history[i].msgSeq = 0;
     }
     for (uint8_t i = 0; i < RELAY_QUEUE_SIZE; i++) {
         queue[i].isActive = false;
     }
 } // Retranslation()
 
 bool Retranslation::isDuplicate(uint8_t senderId, uint8_t msgSeq) {
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         if (history[i].senderId == senderId && history[i].msgSeq == msgSeq) return true; 
     }
     history[head].senderId = senderId;
     history[head].msgSeq = msgSeq;
     head++;
     if (head >= HISTORY_SIZE) head = 0;
     return false; 
 } // isDuplicate()
 
 bool Retranslation::isValidPacket(uint8_t msgType, size_t payloadLen) const {
     if (msgType != MSG_NODE_INFO && msgType != MSG_COORDS && msgType != MSG_LEAVE) return false;
     MessagePolicy policy = getMessagePolicy(msgType);
     return (payloadLen == policy.expectedSize);
 } // isValidPacket()
 
 bool Retranslation::shouldRetransmit(const NavigaHeader& header) const {
     MessagePolicy policy = getMessagePolicy(header.getType());
     if (!policy.isRoutable) return false;
     return header.getTTL() > 1;
 } // shouldRetransmit()
 
 // --- РЕАЛИЗАЦИЯ ОЧЕРЕДИ И SNR-ЗАДЕРЖКИ ---
 
 bool Retranslation::enqueuePacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, float snr) {
     if (payloadLen > MAX_PAYLOAD_SIZE) {
         LOG_WARN("QUEUE", "Payload too large to queue!");
         return false;
     }
 
     for (uint8_t i = 0; i < RELAY_QUEUE_SIZE; i++) {
         if (!queue[i].isActive) {
             queue[i].header = header;
             memcpy(queue[i].payload, payload, payloadLen);
             queue[i].payloadLen = payloadLen;
             
             // Расчет умной задержки (Geographic Routing)
             // Ограничиваем SNR разумными пределами LoRa (-20 дальний предел, +5 сильный сигнал)
             long snrConstrained = (long)snr;
             if (snrConstrained < -20) snrConstrained = -20;
             if (snrConstrained > 5) snrConstrained = 5;
 
             // Чем хуже SNR (ближе к -20), тем меньше базовая задержка (от 100 мс). 
             // Чем лучше SNR (ближе к +5), тем больше базовая задержка (до 1000 мс).
             uint32_t baseDelay = map(snrConstrained, -20, 5, 100, 1000);
             
             // Добавляем рандомную составляющую (джиттер) во избежание точных коллизий
             uint32_t jitter = random(50, 400); 
             uint32_t totalDelayMs = baseDelay + jitter;
             
             queue[i].executeTime = millis() + totalDelayMs;
             queue[i].isActive = true;
             
             LOG_INFO("QUEUE", "Pkt Seq %d queued. SNR: %.1f -> Delay: %d ms", header.msgSeq, snr, totalDelayMs);
             return true;
         }
     }
     LOG_WARN("QUEUE", "Relay queue is FULL! Packet dropped.");
     return false;
 } // enqueuePacket()
 
 bool Retranslation::getReadyPacket(uint8_t myNodeId, NavigaHeader& outHeader, uint8_t* outPayload, size_t& outPayloadLen) {
     uint32_t now = millis();
     for (uint8_t i = 0; i < RELAY_QUEUE_SIZE; i++) {
         if (queue[i].isActive && now >= queue[i].executeTime) {
             outHeader = queue[i].header;
             memcpy(outPayload, queue[i].payload, queue[i].payloadLen);
             outPayloadLen = queue[i].payloadLen;
             
             // Модифицируем заголовок: Уменьшаем TTL на 1, вписываем себя как транзитный узел
             uint8_t currentTTL = outHeader.getTTL();
             outHeader.setTypeAndTTL(static_cast<NavigaMessageType>(outHeader.getType()), currentTTL - 1);
             outHeader.relayId = myNodeId;
             queue[i].isActive = false; // Освобождаем ячейку
             return true; 
         }
     }
     return false;
 } // getReadyPacket()
 
 // --- ЗАГЛУШКИ ДЛЯ БУДУЩЕГО ПОДАВЛЕНИЯ (BROADCAST SUPPRESSION) ---
 
 bool Retranslation::shouldAbortRelay(uint8_t senderId, uint8_t msgSeq) const {
     // В будущем здесь будет логика проверки. 
     // Пока всегда возвращаем false (Слепое затопление - ретранслируем всё).
     return false;
 }
 
 void Retranslation::abortRelay(uint8_t senderId, uint8_t msgSeq) {
     for (uint8_t i = 0; i < RELAY_QUEUE_SIZE; i++) {
         if (queue[i].isActive && queue[i].header.senderId == senderId && queue[i].header.msgSeq == msgSeq) {
             queue[i].isActive = false;
             LOG_INFO("QUEUE", "Relay ABORTED for Seq %d (Suppressed)", msgSeq);
             return; 
         }
     }
 }