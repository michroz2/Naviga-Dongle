/**
 * File: TxManager.h
 * Version: 1.38 
 * Изменение: MAX_PAYLOAD_SIZE увеличен до 24 байт для поддержки кириллицы (Шаг 2).
 * Description: Единый конвейер (Очередь CSMA/CA) для отправки пакетов с поддержкой типа узла.
 */
 #ifndef TX_MANAGER_H
 #define TX_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "RadioManager.h"
 #include "GeoPacker.h"
 
 // Приоритеты для очереди передачи
 enum TxPriority {
     TX_CRITICAL = 0, // Немедленная смена ID при коллизии
     TX_HIGH = 1,     // Отправка собственных координат в движении
     TX_NORMAL = 2,   // Сервисные сообщения (Heartbeat NodeInfo)
     TX_RELAY = 3     // Ретрансляция чужих сообщений (Самый низкий приоритет)
 }; // enum TxPriority
 
 const uint8_t TX_QUEUE_SIZE = 15;    // Максимальный размер очереди
 const uint8_t MAX_PAYLOAD_SIZE = 24; // ИЗМЕНЕНИЕ 1.38: Максимальный размер полезной нагрузки (Payload)
 
 // Структура одной задачи на отправку
 struct TxJob {
     bool isActive;
     TxPriority priority;
     uint32_t readyTime; // Время (millis()), когда задача готова к отправке (учитывает джиттер)
     NavigaHeader header;
     uint8_t payload[MAX_PAYLOAD_SIZE];
     size_t payloadLen;
 }; // struct TxJob 
 
 class TxManager {
 public:
     TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq);
 
     // Методы добавления в очередь разных типов сообщений
     void sendNodeInfo(const char* nodeName, uint8_t nodeType, TxPriority priority = TX_NORMAL);
     void sendCoords(float lat, float lon, TxPriority priority = TX_HIGH);
     
     bool enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, uint32_t delayMs);
     
     // Подавление перехватом (Удаление ретрансляции, если в эфире пойман дубль)
     void abortRelay(uint8_t senderId, uint8_t msgSeq);
     
     // Главный диспетчер очередей
     void processQueue();
 
 private:
     RadioManager& _radio;
     GeoPacker& _packer;
     uint8_t& _myNodeId; // Ссылка на глобальный ID устройства
     uint8_t& _myMsgSeq; // Ссылка на глобальный счетчик пакетов
 
     TxJob _queue[TX_QUEUE_SIZE];
     
     int8_t _activeJobIndex;
     uint32_t _jitterStartTime;
     uint32_t _jitterDelay;
 
     // Внутренний метод постановки в очередь
     bool enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs = 0);
 }; // class TxManager
 
 #endif // TX_MANAGER_H