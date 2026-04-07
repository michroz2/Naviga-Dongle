/**
 * File: Retranslation.cpp
 * Version: 1.18 Изменение: Реализация фильтров для роли Трекера (остановка ретрансляции на бегу, фильтр RELAY/INFO).
 * Description: Реализация класса фильтрации эфира.
 */
 #include "Retranslation.h"
#include "NodeDatabase.h" 
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
 
 bool Retranslation::shouldRetransmit(const NavigaHeader& header, const NodeDatabase& nodeDB, uint8_t myNodeType, float mySpeed) const {
    MessagePolicy policy = getMessagePolicy(header.getType());
    if (!policy.isRoutable) return false;

    if (header.getTTL() <= 1) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: TTL expired.", header.msgSeq);
        return false;
    } 
    
    // --- фильтр количества узлов ---
    if (nodeDB.getActiveNodesCount() < 3) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: Network too small (%d nodes).", header.msgSeq, nodeDB.getActiveNodesCount());
        return false;
    }

    // --- РОЛЕВЫЕ ФИЛЬТРЫ ТРЕКЕРА (Шаг 2) ---
    if (myNodeType == NODE_TRACKER) {
        // Трекер бежит: полностью отключаем функцию радиоудлинителя для экономии батареи и CPU
        if (mySpeed > TRACKER_FAST_SPEED_KMPH) {
            LOG_INFO("RELAY", "Packet Seq %d dropped: We are running Tracker (%.1f km/h).", header.msgSeq, mySpeed);
            return false;
        }
        
        // Трекер идет/стоит: ретранслирует выборочно, отбрасывая "тяжелый" админ-трафик
        if (header.getType() == MSG_NODE_INFO) {
            LOG_INFO("RELAY", "Packet Seq %d dropped: Trackers do not relay NODE_INFO.", header.msgSeq);
            return false;
        }
        
        // Запрещаем Трекеру удлинять стационарную магистраль
        // Проверяем именно того, от кого ФИЗИЧЕСКИ пришел пакет (relayId)
        const NodeRecord* relayer = nodeDB.getNode(header.relayId);
        if (relayer != nullptr && relayer->type == NODE_RELAY) {
            LOG_INFO("RELAY", "Packet Seq %d dropped: Trackers do not relay for RELAY nodes.", header.msgSeq);
            return false;
        }
    }


    if (nodeDB.getCachedMaxDist() < MAX_DIRECT_CONNECT_METERS) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: Group is dense (Max %.1fm).", header.msgSeq, nodeDB.getCachedMaxDist());
        return false;
    }

    // Векторный фильтр (Строгий Географический тупик)
    if (!nodeDB.hasNodesInOppositeDirection(header.relayId)) {
        LOG_INFO("RELAY", "Packet Seq %d dropped: Geographic dead end (No nodes opposite to %d).", header.msgSeq, header.relayId);
        return false;
    }
    
    return true; 
 }