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
 const uint16_t HISTORY_SIZE = 300; // Хранит последние 300 уникальных пакетов
 
 // Структура записи в кольцевом буфере анти-дубликатора
 struct PacketRecord {
     uint8_t senderId; 
     uint8_t msgSeq;   
 };

 // НОВОЕ: Forward Declaration (упреждающее объявление) класса базы узлов
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
     PacketRecord history[HISTORY_SIZE]; // Кольцевой буфер истории
     uint16_t head;                      // Указатель на голову кольцевого буфера
 }; 
 
 #endif // RETRANSLATION_H