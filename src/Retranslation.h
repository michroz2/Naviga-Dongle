/**
 * File: Retranslation.h
 * Version: 1.2.3
 * Description: Заголовочный файл класса ретрансляции и фильтрации эфира.
 * Отвечает за анализ чужих пакетов и принятие решения о ретрансляции.
 */
#ifndef RETRANSLATION_H
#define RETRANSLATION_H

#include <Arduino.h>
#include "NavigaProtocol.h" // Нужен для структур протокола и политик

// Константа из спецификации: размер буфера анти-дубликатора
const uint16_t HISTORY_SIZE = 300;

// Структура "слепка" пакета для истории
struct PacketRecord {
    uint8_t senderId; // Кто изначально создал пакет
    uint8_t msgSeq;   // Уникальный номер этого пакета
}; // struct PacketRecord

class Retranslation {
public:
    Retranslation();

    // Главный метод таможни (Анти-дубликатор). 
    // Возвращает true, если пакет дубликат (отбрасываем).
    // Если пакет новый, запоминает его и возвращает false.
    bool isDuplicate(uint8_t senderId, uint8_t msgSeq);

    // Проверка корректности пакета: разрешен ли тип и совпадает ли размер нагрузки
    bool isValidPacket(uint8_t msgType, size_t payloadLen) const;

    // Проверка: можно ли пускать пакет дальше в эфир? (хватает ли TTL)
    bool shouldRetransmit(const NavigaHeader& header) const;

private:
    PacketRecord history[HISTORY_SIZE]; // Массив-буфер
    uint16_t head;                      // Указатель на текущую ячейку для записи
}; // class Retranslation

#endif // RETRANSLATION_H
