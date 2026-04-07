/**
 * File: Retranslation.h
 * Version: 1.18 Изменение: Добавлены параметры роли и скорости в shouldRetransmit.
 * Description: Заголовочный файл класса фильтрации эфира.
 * Содержит ТОЛЬКО логику валидации и анти-дубликатор. 
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
 // НОВОЕ: Forward Declaration (упреждающее объявление)
class NodeDatabase;

 class Retranslation {
 public:
     Retranslation();
 
     // Методы валидации (Таможня)
     bool isDuplicate(uint8_t senderId, uint8_t msgSeq);
     bool isValidPacket(uint8_t msgType, size_t payloadLen) const;
     
    // Изменение 1.18: Передаем свою роль и скорость для принятия ролевых решений
    bool shouldRetransmit(const NavigaHeader& header, const NodeDatabase& nodeDB, uint8_t myNodeType, float mySpeed) const;

 private:
     PacketRecord history[HISTORY_SIZE]; 
     uint16_t head;                      
 }; 
 
 #endif // RETRANSLATION_H