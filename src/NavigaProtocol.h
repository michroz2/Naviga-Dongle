/**
 * Project: Naviga-Dongle (T-Beam v1.1 + Custom E22)
 * File: NavigaProtocol.h
 * Version: 1.2.4
 * Description: Протокол коммуникации между узлами и политики маршрутизации
 */

 #ifndef NAVIGA_PROTOCOL_H
 #define NAVIGA_PROTOCOL_H
 
 #include <stdint.h>
 
 // Типы сообщений (Биты 7-4 байта Control)
 enum NavigaMessageType : uint8_t {
     MSG_NODE_INFO = 0x01,
     MSG_COORDS    = 0x02,
     MSG_LEAVE     = 0x03
 };
 
 #pragma pack(push, 1) // Отключаем выравнивание для точной побайтовой передачи по радио
 
 // 1. Универсальный заголовок (4 байта)
 struct NavigaHeader {
     uint8_t senderId; 
     uint8_t relayId;  
     uint8_t msgSeq;   
     uint8_t control;  
     
     void setTypeAndTTL(NavigaMessageType type, uint8_t ttl) {
         control = ((type & 0x0F) << 4) | (ttl & 0x0F);
     }
     uint8_t getType() const { return (control >> 4) & 0x0F; }
     uint8_t getTTL() const { return control & 0x0F; }
 };
 
 // 2. Тип 0x01: NODE INFO (12 байт)
 struct PayloadNodeInfo {
     char nodeName[12];
 };
 
 // 3. Тип 0x02: COORDS (4 байта)
 struct PayloadCoords {
     uint16_t latCompressed; 
     uint16_t lonCompressed; 
 };
 
 // 4. Тип 0x03: LEAVE (1 байт)
 struct PayloadLeave {
     uint8_t reason; // 0 - штатное выключение, 1 - разряд батареи, и т.д.
 };
 
 #pragma pack(pop) // Возвращаем стандартное выравнивание компилятора
 
 // --- ПОЛИТИКА МАРШРУТИЗАЦИИ (Справочник) ---
 struct MessagePolicy {
     bool isRoutable;       
     uint8_t expectedSize;  
 };
 
 inline MessagePolicy getMessagePolicy(uint8_t msgType) {
     switch (msgType) {
         case MSG_NODE_INFO: return {true, sizeof(PayloadNodeInfo)};
         case MSG_COORDS:    return {true, sizeof(PayloadCoords)};
         case MSG_LEAVE:     return {true, sizeof(PayloadLeave)}; // Теперь четко 1 байт
         default:            return {false, 0}; 
     } // switch
 } // getMessagePolicy()
 
 #endif // NAVIGA_PROTOCOL_H