/**
 * File: NavigaProtocol.h
 * Version: 1.1.2
 * Description: Добавлены типы узлов (Tracker, Stalker, Relay) в структуру NodeInfo.
 * Изменение: Приведение к новым правилам оформления (комментирование скобок).
 */
 #ifndef NAVIGA_PROTOCOL_H
 #define NAVIGA_PROTOCOL_H
 
 #include <Arduino.h>
 
 // Типы сообщений (NavigaMessageType)
 enum NavigaMessageType : uint8_t {
     MSG_COORDS = 1,     // Пакет с координатами
     MSG_NODE_INFO = 2,  // Пакет с информацией об узле (Имя, Роль)
     MSG_LEAVE = 3       // Сигнал об отключении устройства из сети
 }; // enum NavigaMessageType
 
 // НОВОЕ: Типы узлов (Ролевая модель)
 enum NodeType : uint8_t {
     NODE_TRACKER = 0, // Мобильный, быстро движущийся объект (отключает ретрансляцию, приоритетный TX)
     NODE_STALKER = 1, // Мобильный/умеренный, стандартный участник группы
     NODE_RELAY = 2    // Стационарный ретранслятор (магистраль)
 }; // enum NodeType
 
 // Структура 4-байтного заголовка протокола Naviga (Для упаковки в эфир)
 struct NavigaHeader {
     uint8_t senderId;   // Оригинальный отправитель
     uint8_t relayId;    // Узел, который физически произвел последний прыжок (пересылку)
     uint8_t msgSeq;     // Порядковый номер сообщения отправителя
     uint8_t typeAndTTL; // Комбинированное поле: 4 бита Тип + 4 бита TTL (Time-To-Live)
 
     // Установка комбинированного поля
     void setTypeAndTTL(NavigaMessageType type, uint8_t ttl) {
         typeAndTTL = (type << 4) | (ttl & 0x0F);
     } // setTypeAndTTL()
 
     // Извлечение типа сообщения
     NavigaMessageType getType() const {
         return static_cast<NavigaMessageType>(typeAndTTL >> 4);
     } // getType()
 
     // Извлечение счетчика TTL
     uint8_t getTTL() const {
         return typeAndTTL & 0x0F;
     } // getTTL()
 }; // struct NavigaHeader
 
 // Полезная нагрузка пакета координат (4 байта)
 struct PayloadCoords {
     uint32_t packedCoords; // Сжатые через GeoPacker координаты
 }; // struct PayloadCoords
 
 // Полезная нагрузка пакета NodeInfo (12 байт)
 struct PayloadNodeInfo {
     uint8_t nodeType;
     char nodeName[11]; // Строго 11 байт (+1 байт типа = 12 байт Payload) - Завершается нуль-терминатором
 }; // struct PayloadNodeInfo
 
 // Полезная нагрузка пакета выхода из сети (1 байт)
 struct PayloadLeave {
     uint8_t reason; // Код причины выхода
 }; // struct PayloadLeave
 
 // Структура для возврата политики обработки конкретного типа пакета
 struct MessagePolicy {
     bool isRoutable;     // Подлежит ли ретрансляции
     size_t expectedSize; // Ожидаемый размер полезной нагрузки в байтах
 }; // struct MessagePolicy
 
 // Функция маппинга типа сообщения на его политику обработки
 inline MessagePolicy getMessagePolicy(uint8_t msgType) {
     switch (msgType) {
         case MSG_COORDS:     return {true,  sizeof(PayloadCoords)};
         case MSG_NODE_INFO:  return {true,  sizeof(PayloadNodeInfo)};
         case MSG_LEAVE:      return {false, sizeof(PayloadLeave)};
         default:             return {false, 0};
     } // switch (msgType)
 } // getMessagePolicy()
 
 #endif // NAVIGA_PROTOCOL_H