/**
 * File: Retranslation.h
 * Version: 1.3.0
 * Description: Заголовочный файл класса фильтрации эфира.
 * Теперь содержит ТОЛЬКО логику валидации и анти-дубликатор. 
 * Вся работа с очередями перенесена в TxManager.
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
 
 class Retranslation {
 public:
     Retranslation();
 
     // Методы валидации (Таможня)
     bool isDuplicate(uint8_t senderId, uint8_t msgSeq);
     bool isValidPacket(uint8_t msgType, size_t payloadLen) const;
     bool shouldRetransmit(const NavigaHeader& header) const;
 
 private:
     PacketRecord history[HISTORY_SIZE]; 
     uint16_t head;                      
 }; 
 
 #endif // RETRANSLATION_H