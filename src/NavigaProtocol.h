/**
 * Project: Naviga-Dongle (T-Beam v1.1 + Custom E22)
 * File: NavigaProtocol.h
 * Version: 1.4
 * Description: Проотокол коммуникации между узлами
 */

#ifndef NAVIGA_PROTOCOL_H
#define NAVIGA_PROTOCOL_H

#include <stdint.h>

// Типы сообщений (Биты 7-4 байта Control)
enum NavigaMessageType : uint8_t {
    MSG_NODE_INFO = 0x01,
    MSG_COORDS    = 0x02,
    MSG_LEAVE     = 0x03
};

#pragma pack(push, 1) // Отключаем выравнивание для точной побайтовой передачи по радио

// 1. Универсальный заголовок (4 байта)
struct NavigaHeader {
    uint8_t senderId; // Байт 0: Идентификатор инициатора (1-255)
    uint8_t relayId;  // Байт 1: Идентификатор ретранслятора
    uint8_t msgSeq;   // Байт 2: Порядковый номер сообщения (0-255)
    uint8_t control;  // Байт 3: Биты 7–4: Тип пакета, Биты 3–0: TTL
    
    // Вспомогательный метод для записи типа и TTL в один байт Control
    void setTypeAndTTL(NavigaMessageType type, uint8_t ttl) {
        control = ((type & 0x0F) << 4) | (ttl & 0x0F);
    }
    
    // Вспомогательный метод для чтения типа пакета
    uint8_t getType() const {
        return (control >> 4) & 0x0F;
    }
    
    // Вспомогательный метод для чтения остатка прыжков
    uint8_t getTTL() const {
        return control & 0x0F;
    }
};

// 2. Тип 0x01: NODE INFO (12 байт полезной нагрузки)
struct PayloadNodeInfo {
    char nodeName[12];
};

// 3. Тип 0x02: COORDS (4 байта полезной нагрузки)
// Внимание: сама логика сжатия и распаковки остается в вашем уже работающем классе.
// Здесь только транспортная структура.
struct PayloadCoords {
    uint16_t latCompressed; // 2 младших байта широты
    uint16_t lonCompressed; // 2 младших байта долготы
};

// 4. Тип 0x03: LEAVE (0 байт полезной нагрузки)
// Структура оставлена пустой для консистентности кода. 
// При отправке LEAVE передается только NavigaHeader.
struct PayloadLeave {
    // Empty
};

#pragma pack(pop) // Возвращаем стандартное выравнивание компилятора

#endif // NAVIGA_PROTOCOL_H