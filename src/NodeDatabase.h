/**
 * File: NodeDatabase.h
 * Version: 1.2.1
 * Description: Заголовочный файл локальной базы данных узлов (соседей)
 */
#ifndef NODE_DATABASE_H
#define NODE_DATABASE_H

#include <Arduino.h>

const uint16_t MAX_NODES = 256; 

const uint32_t NODE_TIMEOUT_MS = 300000; // Время "жизни" узла без связи (5 минут)

// Структура данных об одном соседе
struct NodeRecord {
    uint8_t nodeId;
    char name[13];       // 12 символов + нуль-терминатор
    float lat;
    float lon;
    uint32_t lastSeen;   // Время последнего приема пакета (в миллисекундах)
    bool isActive;       // Флаг присутствия узла в сети
}; // struct NodeRecord

class NodeDatabase {
public:
    NodeDatabase();

    // Обновление координат (вызывается при MSG_COORDS)
    void updateNodeCoords(uint8_t id, float lat, float lon);
    
    // Обновление имени узла (вызывается при MSG_NODE_INFO)
    void updateNodeName(uint8_t id, const char* name);
    
    // Удаление узла из базы (вызывается при штатном MSG_LEAVE)
    void removeNode(uint8_t id);
    
    // Очистка устаревших узлов (нужно вызывать периодически в loop)
    void cleanup();
    
    // Получить количество активных узлов
    uint8_t getActiveNodesCount() const;

    // Получить указатель на данные узла (nullptr, если узел не активен)
    const NodeRecord* getNode(uint8_t id) const;

private:
    NodeRecord nodes[MAX_NODES];

}; // class NodeDatabase

#endif // NODE_DATABASE_H