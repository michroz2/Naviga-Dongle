/**
 * Project: Naviga-Dongle (T-Beam v1.1 + Custom E22)
 * File: NavigaProtocol.h
 * Version: 1.2.3
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
     uint8_t senderId; // Байт 0: Идентификатор инициатора (1-255)
     uint8_t relayId;  // Байт 1: Идентификатор ретранслятора
     uint8_t msgSeq;   // Байт 2: Порядковый номер сообщения (0-255)
     uint8_t control;  // Байт 3: Биты 7–4: Тип пакета, Биты 3–0: TTL
     
     // Вспомогательный метод для записи типа и TTL в один байт Control
     void setTypeAndTTL(NavigaMessageType type, uint8_t ttl) {
         control = ((type & 0x0F) << 4) | (ttl & 0x0F);
     }
     
     // Вспомогательный метод для чтения типа пакета
     uint8_t getType() const {
         return (control >> 4) & 0x0F;
     }
     
     // Вспомогательный метод для чтения остатка прыжков
     uint8_t getTTL() const {
         return control & 0x0F;
     }
 };
 
 // 2. Тип 0x01: NODE INFO (12 байт полезной нагрузки)
 struct PayloadNodeInfo {
     char nodeName[12];
 };
 
 // 3. Тип 0x02: COORDS (4 байта полезной нагрузки)
 struct PayloadCoords {
     uint16_t latCompressed; // 2 младших байта широты
     uint16_t lonCompressed; // 2 младших байта долготы
 };
 
 // 4. Тип 0x03: LEAVE (0 байт полезной нагрузки)
 struct PayloadLeave {
    uint8_t reason; // 0 - штатное выключение, 1 - разряд батареи, и т.д.
 };
 
 #pragma pack(pop) // Возвращаем стандартное выравнивание компилятора
 
 // --- ПОЛИТИКА МАРШРУТИЗАЦИИ (Справочник) ---
 struct MessagePolicy {
     bool isRoutable;       // Разрешено ли ретранслировать этот тип пакета?
     uint8_t expectedSize;  // Ожидаемый размер полезной нагрузки (в байтах)
 };
 
 // Статический метод-справочник для валидации пакетов по типу
 inline MessagePolicy getMessagePolicy(uint8_t msgType) {
     switch (msgType) {
         case MSG_NODE_INFO: 
             return {true, sizeof(PayloadNodeInfo)};
         case MSG_COORDS:    
             return {true, sizeof(PayloadCoords)};
         case MSG_LEAVE:     
             return {true, sizeof(PayloadLeave)}; 
         default:            
             return {false, 0}; // Неизвестный тип - сбрасываем
     } // switch
 } // getMessagePolicy()
 
 #endif // NAVIGA_PROTOCOL_H
 