/**
 * File: Retranslation.cpp
 * Version: 1.12 Добавлена оценкf кэшированных расстояний
 * Description: Реализация класса фильтрации эфира.
 */
 #include "Retranslation.h"
#include "NodeDatabase.h" // НОВОЕ: Подключаем базу физически только в .cpp
#include "logger.h"
#include "configuration.h"

 Retranslation::Retranslation() {
     head = 0;
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         history[i].senderId = 0;
         history[i].msgSeq = 0;
     }
 } 
 
 bool Retranslation::isDuplicate(uint8_t senderId, uint8_t msgSeq) {
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         if (history[i].senderId == senderId && history[i].msgSeq == msgSeq) return true; 
     }
     history[head].senderId = senderId;
     history[head].msgSeq = msgSeq;
     head++;
     if (head >= HISTORY_SIZE) head = 0;
     return false; 
 } 
 
 bool Retranslation::isValidPacket(uint8_t msgType, size_t payloadLen) const {
     if (msgType != MSG_NODE_INFO && msgType != MSG_COORDS && msgType != MSG_LEAVE) return false;
     MessagePolicy policy = getMessagePolicy(msgType);
     return (payloadLen == policy.expectedSize);
 } 
 
 bool Retranslation::shouldRetransmit(const NavigaHeader& header, const NodeDatabase& nodeDB) const {
    // Тип пакета - ретранслируемый?
    MessagePolicy policy = getMessagePolicy(header.getType());
     if (!policy.isRoutable) return false;

    // Если TTL пакета исчерпан (остался 1 прыжок, который пакет только что совершил до нас),
    // пакет считается доставленным, но ретранслировать его дальше нельзя.
    if (header.getTTL() <= 1) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: TTL expired.", header.msgSeq);
        return false;
    } // if (header.getTTL() <= 1)
    
    // --- Географические фильтры (мгновенное чтение) ---
    // Фильтр 1: Маленькая сеть
    if (nodeDB.getActiveNodesCount() < 3) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: Network too small (%d nodes).", header.msgSeq, nodeDB.getActiveNodesCount());
        return false;
    }
    // Фильтр 2: Плотная группа (прямая видимость)
    if (nodeDB.getCachedMaxDist() < MAX_DIRECT_CONNECT_METERS) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: Group is dense (Max %.1fm).", header.msgSeq, nodeDB.getCachedMaxDist());
        return false;
    }
    
    return true; 
 } // Retranslation::shouldRetransmit()