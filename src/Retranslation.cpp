/**
 * File: Retranslation.cpp
 * Version 1.2.2.1
 * Description: Реализация класса ретрансляции (кольцевой буфер анти-дубликатов).
 */
#include "Retranslation.h"

Retranslation::Retranslation() {
    head = 0;
    // Инициализируем буфер нулями при старте
    for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
        history[i].senderId = 0;
        history[i].msgSeq = 0;
    } // for
} // Retranslation()

bool Retranslation::isDuplicate(uint8_t senderId, uint8_t msgSeq) {
    // 1. Проверяем, видели ли мы этот пакет ранее
    for (uint16_t i = 0; i < HISTORY_SIZE; i++) {
        // Если идентификатор создателя и номер сообщения совпадают
        if (history[i].senderId == senderId && history[i].msgSeq == msgSeq) {
            return true; // Пакет найден в истории, это дубликат!
        } // if
    } // for

    // 2. Если мы дошли сюда, значит пакет уникальный (новый).
    // Записываем его данные в ячейку, на которую сейчас смотрит указатель head.
    history[head].senderId = senderId;
    history[head].msgSeq = msgSeq;

    // 3. Сдвигаем указатель вперед для следующего пакета
    head++;
    
    // 4. Если указатель вышел за пределы массива, закольцовываем его на начало
    if (head >= HISTORY_SIZE) {
        head = 0;
    } // if limit

    return false; // Сообщаем системе, что пакет новый и готов к обработке
} // isDuplicate()