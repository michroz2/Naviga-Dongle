/**
 * File: NodeDatabase.cpp
 * Version: 1.16 Изменение: Реализация защиты excludeNodeId в сборщике мусора cleanup.
 * Description: Реализация базы данных узлов.
 */
 #include "NodeDatabase.h"
#include "logger.h"
#include "configuration.h"
#include <string.h>
 
 NodeDatabase::NodeDatabase() {
    _activeNodesCount = 0;
    _cachedMaxDist = 0.0f;
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
         return nullptr; // "Честный" геттер: не меняем базу, если узел неактивен
     } 
     return &nodes[nodeId];
 } // NodeDatabase::getNode()

 void NodeDatabase::addNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     
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
         _activeNodesCount++;            
         snprintf(nodes[nodeId].nodeName, sizeof(nodes[nodeId].nodeName), "Node-%d", nodeId);
     } 
 } // NodeDatabase::addNode()

// Метод только для чтения статуса
bool NodeDatabase::isNodeActive(uint8_t nodeId) const {
    if (nodeId == 0 || nodeId >= MAX_NODES) return false;
    return nodes[nodeId].isActive;
} // NodeDatabase::isNodeActive()

 void NodeDatabase::updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) {
         addNode(nodeId);
     }
     nodes[nodeId].type = nodeType;
     strncpy(nodes[nodeId].nodeName, name, sizeof(nodes[nodeId].nodeName) - 1);
     nodes[nodeId].nodeName[sizeof(nodes[nodeId].nodeName) - 1] = '\0';
     nodes[nodeId].lastSeen = millis();
 } // NodeDatabase::updateNodeInfo()
 
 void NodeDatabase::updateNodeSNR(uint8_t nodeId, float snr) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) {
         addNode(nodeId);
     }
     nodes[nodeId].snr = snr;
     nodes[nodeId].lastSeen = millis(); 
 } // NodeDatabase::updateNodeSNR()
 
 void NodeDatabase::updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) {
         addNode(nodeId);
     }
     nodes[nodeId].distance = dist;
     nodes[nodeId].azimuth = azmt;
 } // NodeDatabase::updateNodeDistanceAzimuth()
 
 void NodeDatabase::updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) {
         addNode(nodeId);
     }
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
     if (_activeNodesCount > 0) _activeNodesCount--; 
    } // NodeDatabase::removeNode()
 
    void NodeDatabase::cleanup(uint8_t excludeNodeId) {
        uint32_t currentMillis = millis();
        for (int i = 1; i < MAX_NODES; i++) {
            // Защита локального узла от очистки по таймауту
            if (i == excludeNodeId) continue;

            if (nodes[i].isActive && (currentMillis - nodes[i].lastSeen > NODE_TIMEOUT_MS)) {
                nodes[i].isActive = false;
                if (_activeNodesCount > 0) _activeNodesCount--;
                LOG_INFO("SYS", "Node %d removed by timeout", i);
            }
        }
    } // NodeDatabase::cleanup()
 
// --- Быстрые геттеры из кэша (O(1)) ---

uint8_t NodeDatabase::getActiveNodesCount() const {
    return _activeNodesCount;
}

float NodeDatabase::getCachedMaxDist() const {
    return _cachedMaxDist;
}

void NodeDatabase::updateTopology() {
    uint8_t count = 0;
    float maxD = 0.0f;

    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            count++;
            if (nodes[i].distance > maxD) {
                maxD = nodes[i].distance;
            }
        }
    }
    
    _activeNodesCount = count;
    _cachedMaxDist = maxD;
    
    LOG_INFO("SYS", "Topology sync: Nodes: %d, MaxDist: %.1fm", _activeNodesCount, _cachedMaxDist);
}