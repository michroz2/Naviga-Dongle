/**
 * File: TxManager.h
 * Version: 1.1.0
 * Description: Единый MAC-диспетчер. Формирует пакеты, управляет 
 * приоритетной очередью и реализует алгоритм CSMA/CA (Jitter).
 */
 #ifndef TX_MANAGER_H
 #define TX_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "RadioManager.h"
 #include "GeoPacker.h"
 
 enum TxPriority {
     TX_CRITICAL = 0, // Экстренно (коллизия)
     TX_HIGH = 1,     // Свои регулярные координаты
     TX_NORMAL = 2,   // Свой служебный трафик (медленный пульс)
     TX_RELAY = 3     // Чужие пакеты на ретрансляцию
 };
 
 const uint8_t TX_QUEUE_SIZE = 15;
 const uint8_t MAX_PAYLOAD_SIZE = 16; 
 
 struct TxJob {
     bool isActive;
     TxPriority priority;
     uint32_t readyTime; // Время, после которого пакет готов встать в очередь (для SNR задержки)
     NavigaHeader header;
     uint8_t payload[MAX_PAYLOAD_SIZE];
     size_t payloadLen;
 };
 
 class TxManager {
 public:
     TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq);
 
     // Интерфейс для формирования собственных пакетов
     void sendNodeInfo(const char* nodeName, TxPriority priority = TX_NORMAL);
     void sendCoords(float lat, float lon, TxPriority priority = TX_HIGH);
     
     // Интерфейс для приема чужих пакетов на ретрансляцию
     bool enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, float snr);
 
     // Управление подавлением (Broadcast Suppression)
     void abortRelay(uint8_t senderId, uint8_t msgSeq);
 
     // Главный процессор очереди (вызывается в loop)
     void processQueue();
 
 private:
     RadioManager& _radio;
     GeoPacker& _packer;
     uint8_t& _myNodeId;
     uint8_t& _myMsgSeq;
 
     TxJob _queue[TX_QUEUE_SIZE];
     
     // Переменные конечного автомата (State Machine) для Jitter
     int8_t _activeJobIndex;
     uint32_t _jitterStartTime;
     uint32_t _jitterDelay;
 
     bool enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs = 0);
 };
 
 #endif // TX_MANAGER_H