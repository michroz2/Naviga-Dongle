/**
 * File: Retranslation.h
 * Version: 1.2.7
 * Description: Заголовочный файл класса ретрансляции и фильтрации эфира.
 * Содержит анти-дубликатор и умную очередь отложенной передачи (с SNR-задержкой).
 */
 #ifndef RETRANSLATION_H
 #define RETRANSLATION_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 
 // --- НАСТРОЙКИ АНТИ-ДУБЛИКАТОРА ---
 const uint16_t HISTORY_SIZE = 300;
 
 struct PacketRecord {
     uint8_t senderId; 
     uint8_t msgSeq;   
 };
 
 // --- НАСТРОЙКИ ОЧЕРЕДИ РЕТРАНСЛЯЦИИ ---
 const uint8_t RELAY_QUEUE_SIZE = 10; // Максимум пакетов в очереди
 const uint8_t MAX_PAYLOAD_SIZE = 16; // С запасом под самую большую структуру (NODE_INFO = 12)
 
 struct RelayJob {
     bool isActive;           // Занята ли эта ячейка
     uint32_t executeTime;    // Время (millis), когда пора отправлять
     NavigaHeader header;     // Копия оригинального заголовка
     uint8_t payload[MAX_PAYLOAD_SIZE]; // Копия полезной нагрузки
     size_t payloadLen;       // Длина полезной нагрузки
 };
 
 class Retranslation {
 public:
     Retranslation();
 
     // Методы валидации (Таможня)
     bool isDuplicate(uint8_t senderId, uint8_t msgSeq);
     bool isValidPacket(uint8_t msgType, size_t payloadLen) const;
     bool shouldRetransmit(const NavigaHeader& header) const;
 
     // --- ОЧЕРЕДЬ И УМНАЯ ЗАДЕРЖКА ---
     // Поместить пакет в очередь. Задержка высчитывается на основе SNR.
     bool enqueuePacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen, float snr);
     
     // Проверить, подошло ли время отправки какого-либо пакета.
     bool getReadyPacket(uint8_t myNodeId, NavigaHeader& outHeader, uint8_t* outPayload, size_t& outPayloadLen);
 
     // --- УПРАВЛЕНИЕ ПОДАВЛЕНИЕМ (BROADCAST SUPPRESSION) ---
     // Проверка: нужно ли отменить ретрансляцию (пока всегда возвращает false)
     bool shouldAbortRelay(uint8_t senderId, uint8_t msgSeq) const;
     
     // Само удаление пакета из очереди (если мы услышали, что кто-то уже его переслал)
     void abortRelay(uint8_t senderId, uint8_t msgSeq);
 
 private:
     PacketRecord history[HISTORY_SIZE]; 
     uint16_t head;                      
     RelayJob queue[RELAY_QUEUE_SIZE]; // Массив задач на отправку
 }; 
 
 #endif // RETRANSLATION_H