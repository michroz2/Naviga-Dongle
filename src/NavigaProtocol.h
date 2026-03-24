/**
 * File: NavigaProtocol.h
 * Version: 1.1.2
 * Description: Добавлены типы узлов (Tracker, Stalker, Relay) в структуру NodeInfo.
 * Изменение: Приведение к новым правилам оформления (комментирование скобок).
 */
 #ifndef NAVIGA_PROTOCOL_H
 #define NAVIGA_PROTOCOL_H
 
 #include <Arduino.h>
 
 enum NavigaMessageType : uint8_t {
     MSG_COORDS = 1,
     MSG_NODE_INFO = 2,
     MSG_LEAVE = 3
 }; // enum NavigaMessageType
 
 // НОВОЕ: Типы узлов
 enum NodeType : uint8_t {
     NODE_TRACKER = 0, // Мобильный, приоритетно шлет координаты
     NODE_STALKER = 1, // Мобильный/умеренный, шлет координаты, умно ретранслирует
     NODE_RELAY = 2    // Стационарный, ретранслирует всё
 }; // enum NodeType
 
 struct NavigaHeader {
     uint8_t senderId;
     uint8_t relayId;
     uint8_t msgSeq;
     uint8_t typeAndTTL; 
 
     void setTypeAndTTL(NavigaMessageType type, uint8_t ttl) {
         typeAndTTL = (type << 4) | (ttl & 0x0F);
     } // setTypeAndTTL()
 
     NavigaMessageType getType() const {
         return static_cast<NavigaMessageType>(typeAndTTL >> 4);
     } // getType()
 
     uint8_t getTTL() const {
         return typeAndTTL & 0x0F;
     } // getTTL()
 }; // struct NavigaHeader
 
 struct PayloadCoords {
     uint32_t packedCoords;
 }; // struct PayloadCoords
 
 struct PayloadNodeInfo {
     uint8_t nodeType;
     char nodeName[11]; // Строго 11 байт (+1 байт типа = 12 байт Payload)
 }; // struct PayloadNodeInfo
 
 struct PayloadLeave {
     uint8_t reason;
 }; // struct PayloadLeave
 
 struct MessagePolicy {
     bool isRoutable;
     size_t expectedSize;
 }; // struct MessagePolicy
 
 inline MessagePolicy getMessagePolicy(uint8_t msgType) {
     switch (msgType) {
         case MSG_COORDS:     return {true,  sizeof(PayloadCoords)};
         case MSG_NODE_INFO:  return {true,  sizeof(PayloadNodeInfo)};
         case MSG_LEAVE:      return {false, sizeof(PayloadLeave)};
         default:             return {false, 0};
     } // switch (msgType)
 } // getMessagePolicy()
 
 #endif // NAVIGA_PROTOCOL_H