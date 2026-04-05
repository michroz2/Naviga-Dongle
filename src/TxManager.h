/**
 * File: TxManager.h
 * Version: 1.10 Изменение: Добавлено поле rxSnr в TxJob для расчета джиттера при ретрансляции.
 * Description: Единый конвейер для отправки пакетов с поддержкой типа узла.
 */
 #ifndef TX_MANAGER_H
 #define TX_MANAGER_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 #include "RadioManager.h"
 #include "GeoPacker.h"
 
 enum TxPriority {
     TX_CRITICAL = 0, 
     TX_HIGH = 1,     
     TX_NORMAL = 2,   
     TX_RELAY = 3     
 }; // enum TxPriority
 
 const uint8_t TX_QUEUE_SIZE = 15;
 const uint8_t MAX_PAYLOAD_SIZE = 16; 
 
 struct TxJob {
    bool isActive;
    TxPriority priority;
    uint32_t readyTime; 
    NavigaHeader header;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    size_t payloadLen;
    float rxSnr;            //Сохраняем SNR принятого пакета для расчета задержки ретрансляции
 }; // struct TxJob 

 class TxManager {
 public:
     TxManager(RadioManager& radio, GeoPacker& packer, uint8_t& nodeId, uint8_t& msgSeq);
 
     void sendNodeInfo(const char* nodeName, uint8_t nodeType, TxPriority priority = TX_NORMAL);
     void sendCoords(float lat, float lon, TxPriority priority = TX_HIGH);
     
     bool enqueueRelay(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, float snr);
     void abortRelay(uint8_t senderId, uint8_t msgSeq);
     void processQueue();
 
 private:
     RadioManager& _radio;
     GeoPacker& _packer;
     uint8_t& _myNodeId;
     uint8_t& _myMsgSeq;
 
     TxJob _queue[TX_QUEUE_SIZE];
     
     int8_t _activeJobIndex;
     uint32_t _jitterStartTime;
     uint32_t _jitterDelay;
 
     bool enqueue(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, TxPriority priority, uint32_t delayMs = 0);
 }; // class TxManager
 
 #endif // TX_MANAGER_H