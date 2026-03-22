/**
 * File: NodeDatabase.cpp
 * Version: 1.2.9
 * Description: Реализация локальной базы данных узлов.
 */
 #include "NodeDatabase.h"
 #include <string.h>
 
 NodeDatabase::NodeDatabase() {
     for (int i = 0; i < MAX_NODES; i++) {
         nodes[i].isActive = false;
         nodes[i].nodeId = i; 
         memset(nodes[i].name, 0, sizeof(nodes[i].name));
         nodes[i].lat = 0.0f;
         nodes[i].lon = 0.0f;
         nodes[i].packedCoords = 0; // Инициализируем нулем
     } // for
 } // NodeDatabase()
 
 void NodeDatabase::updateNodeCoords(uint8_t id, float lat, float lon, uint32_t packedCoords, bool updateTime) {
     if (!nodes[id].isActive) {
         nodes[id].isActive = true;
         snprintf(nodes[id].name, sizeof(nodes[id].name), "Node-%d", id);
     } 
     
     nodes[id].lat = lat;
     nodes[id].lon = lon;
     nodes[id].packedCoords = packedCoords; // Всегда пишем сырые данные
     
     // Обновляем lastSeen только если это свежий пакет из эфира
     if (updateTime) {
         nodes[id].lastSeen = millis();
     }
 } // updateNodeCoords()
 
 void NodeDatabase::updateNodeName(uint8_t id, const char* name) {
     if (!nodes[id].isActive) {
         nodes[id].isActive = true;
         nodes[id].lat = 0.0f;
         nodes[id].lon = 0.0f;
         nodes[id].packedCoords = 0;
     } 
     
     strncpy(nodes[id].name, name, sizeof(nodes[id].name) - 1);
     nodes[id].name[sizeof(nodes[id].name) - 1] = '\0';
     nodes[id].lastSeen = millis();
 } // updateNodeName()
 
 void NodeDatabase::removeNode(uint8_t id) {
     nodes[id].isActive = false;
 } // removeNode()
 
 void NodeDatabase::cleanup() {
     uint32_t currentMillis = millis();
     for (int i = 0; i < MAX_NODES; i++) {
         if (nodes[i].isActive) {
             if (currentMillis - nodes[i].lastSeen > NODE_TIMEOUT_MS) {
                 nodes[i].isActive = false;
             } 
         } 
     } 
 } // cleanup()
 
 uint8_t NodeDatabase::getActiveNodesCount() const {
     uint8_t count = 0;
     for (int i = 1; i < MAX_NODES; i++) {
         if (nodes[i].isActive) count++;
     } 
     return count;
 } // getActiveNodesCount()
 
 const NodeRecord* NodeDatabase::getNode(uint8_t id) const {
     if (nodes[id].isActive) {
         return &nodes[id]; 
     } 
     return nullptr;
 } // getNode()