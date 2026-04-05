/**
 * File: NodeDatabase.cpp
 * Version: 1.7 Изменение: Реализация метода isNodeActive.
 * Description: Реализация базы данных узлов.
 */
 #include "NodeDatabase.h"
 #include "logger.h"
 #include <string.h>

 NodeDatabase::NodeDatabase() {
     for (int i = 0; i < MAX_NODES; i++) {
         nodes[i].nodeId = i;
         nodes[i].isActive = false;
         nodes[i].lastSeen = 0;
         nodes[i].packedCoords = 0;
         nodes[i].lat = 0.0f;
         nodes[i].lon = 0.0f;
         nodes[i].snr = -100.0f; 
         nodes[i].distance = 0.0f; 
         nodes[i].azimuth = 0.0f;  
         nodes[i].type = NODE_STALKER; // Дефолтный тип
         snprintf(nodes[i].nodeName, sizeof(nodes[i].nodeName), "Node-%d", i);
     } // for (int i = 0; i < MAX_NODES; i++)
 } // NodeDatabase::NodeDatabase()
 
 const NodeRecord* NodeDatabase::getNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return nullptr;
     
     if (!nodes[nodeId].isActive) {
         nodes[nodeId].isActive = true;
         nodes[nodeId].lastSeen = millis();
         nodes[nodeId].packedCoords = 0;
         nodes[nodeId].lat = 0.0f;
         nodes[nodeId].lon = 0.0f;
         nodes[nodeId].snr = -100.0f; 
         nodes[nodeId].distance = 0.0f;
         nodes[nodeId].azimuth = 0.0f;
         nodes[nodeId].type = NODE_STALKER;
         snprintf(nodes[nodeId].nodeName, sizeof(nodes[nodeId].nodeName), "Node-%d", nodeId);
     } // if (!nodes[nodeId].isActive)
     return &nodes[nodeId];
 } // NodeDatabase::getNode()

// Метод только для чтения статуса
bool NodeDatabase::isNodeActive(uint8_t nodeId) const {
    if (nodeId == 0 || nodeId >= MAX_NODES) return false;
    return nodes[nodeId].isActive;
} // NodeDatabase::isNodeActive()

 void NodeDatabase::updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     getNode(nodeId);
     nodes[nodeId].type = nodeType;
     strncpy(nodes[nodeId].nodeName, name, sizeof(nodes[nodeId].nodeName) - 1);
     nodes[nodeId].nodeName[sizeof(nodes[nodeId].nodeName) - 1] = '\0';
     nodes[nodeId].lastSeen = millis();
 } // NodeDatabase::updateNodeInfo()
 
 void NodeDatabase::updateNodeSNR(uint8_t nodeId, float snr) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     const NodeRecord* node = getNode(nodeId); 
     if (node != nullptr) {
         nodes[nodeId].snr = snr;
         nodes[nodeId].lastSeen = millis(); 
     } // if (node != nullptr)
 } // NodeDatabase::updateNodeSNR()
 
 void NodeDatabase::updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     const NodeRecord* node = getNode(nodeId);
     if (node != nullptr) {
         nodes[nodeId].distance = dist;
         nodes[nodeId].azimuth = azmt;
     } // if (node != nullptr)
 } // NodeDatabase::updateNodeDistanceAzimuth()
 
 void NodeDatabase::updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     getNode(nodeId); 
     nodes[nodeId].lat = lat;
     nodes[nodeId].lon = lon;
     if (packed != 0) {
         nodes[nodeId].packedCoords = packed;
     } // if (packed != 0)
     if (updateTimer) {
         nodes[nodeId].lastSeen = millis();
     } // if (updateTimer)
 } // NodeDatabase::updateNodeCoords()
 
 void NodeDatabase::removeNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     nodes[nodeId].isActive = false;
 } // NodeDatabase::removeNode()
 
 void NodeDatabase::cleanup() {
    uint32_t now = millis();
    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive && (now - nodes[i].lastSeen > NODE_TIMEOUT_MS)) {
            nodes[i].isActive = false;
            
            // Логируем существенное действие системы
            LOG_INFO("ACTION", "Node %d (%s) deleted due to timeout", i, nodes[i].nodeName);
        } // if (nodes[i].isActive && ...)
    } // for (int i = 1; i < MAX_NODES; i++)
} // NodeDatabase::cleanup()
 
 uint8_t NodeDatabase::getActiveNodesCount() const {
     uint8_t count = 0;
     for (int i = 1; i < MAX_NODES; i++) {
         if (nodes[i].isActive) count++;
     } // for (int i = 1; i < MAX_NODES; i++)
     return count;
 } // NodeDatabase::getActiveNodesCount()