/**
 * File: NodeDatabase.cpp
 * Version: 1.2.1
 * Description: Реализация локальной базы данных узлов (соседей)
 */
#include "NodeDatabase.h"
#include <string.h>

NodeDatabase::NodeDatabase() {
    for (int i = 0; i < MAX_NODES; i++) {
        nodes[i].isActive = false;
        nodes[i].nodeId = i; // Сразу жестко прописываем ID, равный индексу ячейки
        memset(nodes[i].name, 0, sizeof(nodes[i].name));
    } // for
} // NodeDatabase()

void NodeDatabase::updateNodeCoords(uint8_t id, float lat, float lon) {
    // Если получаем пакет от этого ID впервые
    if (!nodes[id].isActive) {
        nodes[id].isActive = true;
        // Даем дефолтное имя, пока не получим пакет NODE_INFO
        snprintf(nodes[id].name, sizeof(nodes[id].name), "Node-%d", id);
    } // if (!isActive)
    
    nodes[id].lat = lat;
    nodes[id].lon = lon;
    nodes[id].lastSeen = millis();
} // updateNodeCoords()

void NodeDatabase::updateNodeName(uint8_t id, const char* name) {
    if (!nodes[id].isActive) {
        nodes[id].isActive = true;
        nodes[id].lat = 0.0f;
        nodes[id].lon = 0.0f;
    } // if (!isActive)
    
    // Безопасное копирование строки, чтобы не выйти за пределы массива
    strncpy(nodes[id].name, name, sizeof(nodes[id].name) - 1);
    nodes[id].name[sizeof(nodes[id].name) - 1] = '\0';
    nodes[id].lastSeen = millis();
} // updateNodeName()

void NodeDatabase::removeNode(uint8_t id) {
    // Просто выключаем флаг активности по прямому индексу
    nodes[id].isActive = false;
} // removeNode()
// КОНЕЦ ИЗМЕНЕНИЯ

void NodeDatabase::cleanup() {
    uint32_t currentMillis = millis();
    for (int i = 0; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            // Защита от переполнения millis() (переход через 0 каждые 50 дней)
            if (currentMillis - nodes[i].lastSeen > NODE_TIMEOUT_MS) {
                nodes[i].isActive = false;
            } // if timeout
        } // if isActive
    } // for
} // cleanup()

uint8_t NodeDatabase::getActiveNodesCount() const {
    uint8_t count = 0;
    // Начинаем с 1, так как ID=0 зарезервирован и не используется
    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            count++;
        } // if isActive
    } // for
    return count;
} // getActiveNodesCount()


const NodeRecord* NodeDatabase::getNode(uint8_t id) const {
    if (nodes[id].isActive) {
        return &nodes[id]; // Возвращаем адрес ячейки напрямую
    } // if isActive
    return nullptr;
} // getNode()
