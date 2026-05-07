/**
 * File: Retranslation.cpp
 * Version: 1.19 Изменение: Отключение векторного фильтра для роли NODE_RELAY (Шаг 2).
 * Description: Реализация класса фильтрации эфира ("Таможня эфира").
 */
 #include "Retranslation.h"
 #include "NodeDatabase.h" 
 #include "logger.h"
 #include "configuration.h"
 
 Retranslation::Retranslation() {
     // Инициализация кольцевого буфера анти-дубликатора
     head = 0;
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         history[i].senderId = 0;
         history[i].msgSeq = 0;
     }
 } 
  
 // Анти-дубликатор: Проверяет, был ли этот пакет (ID + номер) обработан ранее.
 bool Retranslation::isDuplicate(uint8_t senderId, uint8_t msgSeq) {
     for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
         if (history[i].senderId == senderId && history[i].msgSeq == msgSeq) return true; 
     }
     
     // Если пакет новый, записываем его в кольцевой буфер
     history[head].senderId = senderId;
     history[head].msgSeq = msgSeq;
     head++;
     if (head >= HISTORY_SIZE) head = 0;
     return false; 
 } 
  
 // Базовая валидация пакета по размеру полезной нагрузки
 bool Retranslation::isValidPacket(uint8_t msgType, size_t payloadLen) const {
     if (msgType != MSG_NODE_INFO && msgType != MSG_COORDS && msgType != MSG_LEAVE) return false;
     MessagePolicy policy = getMessagePolicy(msgType);
     return (payloadLen == policy.expectedSize);
 } 
  
 // Главный метод: Решение о том, должен ли локальный узел ретранслировать пакет
 bool Retranslation::shouldRetransmit(const NavigaHeader& header, const NodeDatabase& nodeDB, uint8_t myNodeType, float mySpeed) const {
     // Проверка: маршрутизируется ли в принципе данный тип сообщений
     MessagePolicy policy = getMessagePolicy(header.getType());
     if (!policy.isRoutable) return false;
 
     // Проверка Time-to-Live (TTL): Если жизни не осталось, пакет сбрасывается
     if (header.getTTL() <= 1) {
         LOG_INFO("RELAY", "Packet Seq %d dropped: TTL expired.", header.msgSeq);
         return false;
     } 
     
     // --- фильтр количества узлов ---
     // Если в сети мало узлов (меньше 3), ретрансляция не имеет смысла (все слышат друг друга напрямую)
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
         
         // Трекер идет/стоит: ретранслирует выборочно, отбрасывая "тяжелый" админ-трафик (NodeInfo)
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
 
     // Фильтр Компактной группы: Если максимальное расстояние в группе меньше 200м, ретрансляция не нужна
     if (nodeDB.getCachedMaxDist() < MAX_DIRECT_CONNECT_METERS) {
         LOG_INFO("RELAY", "Packet Seq %d dropped: Group is dense (Max %.1fm).", header.msgSeq, nodeDB.getCachedMaxDist());
         return false;
     }
 
     // Векторный фильтр (Строгий Географический тупик)
     // Проверяет, есть ли узлы "за спиной" относительно источника пакета
     // ИЗМЕНЕНИЕ 1.19: Отключение векторного фильтра для роли NODE_RELAY (Магистраль светит во все стороны)
     if (myNodeType != NODE_RELAY) {
         if (!nodeDB.hasNodesInOppositeDirection(header.relayId)) {
             LOG_INFO("RELAY", "Packet Seq %d dropped: Geographic dead end (No nodes opposite to %d).", header.msgSeq, header.relayId);
             return false;
         }
     }
     
     // Если пакет прошел все фильтры, возвращаем true
     return true; 
 } //Retranslation.cpp