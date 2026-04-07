/**
 * File: NodeDatabase.cpp
 * Version: 1.17 Изменение: Реализация сбора квадрантов в updateTopology и метода hasNodesInOppositeDirection.
 * Description: Реализация базы данных узлов.
 */
 #include "NodeDatabase.h"
#include "logger.h"
#include "configuration.h"
#include <string.h>
 
 NodeDatabase::NodeDatabase() {
    _activeNodesCount = 0;
    _cachedMaxDist = 0.0f;
    for (int i=0; i<4; i++) _quadrantNodes[i] = 0;
    
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
         nodes[i].type = NODE_STALKER; 
         snprintf(nodes[i].nodeName, sizeof(nodes[i].nodeName), "Node-%d", i);
     } 
 } 
 
 const NodeRecord* NodeDatabase::getNode(uint8_t nodeId) const {
    if (nodeId == 0 || nodeId >= MAX_NODES) return nullptr;
    if (!nodes[nodeId].isActive) return nullptr; 
     return &nodes[nodeId];
 } 

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
 } 

bool NodeDatabase::isNodeActive(uint8_t nodeId) const {
    if (nodeId == 0 || nodeId >= MAX_NODES) return false;
    return nodes[nodeId].isActive;
} 

 void NodeDatabase::updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].type = nodeType;
     strncpy(nodes[nodeId].nodeName, name, sizeof(nodes[nodeId].nodeName) - 1);
     nodes[nodeId].nodeName[sizeof(nodes[nodeId].nodeName) - 1] = '\0';
     nodes[nodeId].lastSeen = millis();
 } 
 
 void NodeDatabase::updateNodeSNR(uint8_t nodeId, float snr) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].snr = snr;
     nodes[nodeId].lastSeen = millis(); 
 } 
 
 void NodeDatabase::updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].distance = dist;
     nodes[nodeId].azimuth = azmt;
 } 
 
 void NodeDatabase::updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].lat = lat;
     nodes[nodeId].lon = lon;
     if (packed != 0) nodes[nodeId].packedCoords = packed;
     if (updateTimer) nodes[nodeId].lastSeen = millis();
 } 
 
 void NodeDatabase::removeNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     nodes[nodeId].isActive = false;
     if (_activeNodesCount > 0) _activeNodesCount--; 
    } 
 
    void NodeDatabase::cleanup(uint8_t excludeNodeId) {
        uint32_t currentMillis = millis();
        for (int i = 1; i < MAX_NODES; i++) {
            if (i == excludeNodeId) continue;
            if (nodes[i].isActive && (currentMillis - nodes[i].lastSeen > NODE_TIMEOUT_MS)) {
                nodes[i].isActive = false;
                if (_activeNodesCount > 0) _activeNodesCount--;
                LOG_INFO("SYS", "Node %d removed by timeout", i);
            }
        }
    } 
 
uint8_t NodeDatabase::getActiveNodesCount() const { return _activeNodesCount; }
float NodeDatabase::getCachedMaxDist() const { return _cachedMaxDist; }

// НОВОЕ: Реализация метода проверки противоположного квадранта
bool NodeDatabase::hasNodesInOppositeDirection(uint8_t referenceNodeId) const {
    if (referenceNodeId == 0 || referenceNodeId >= MAX_NODES) return true;
    if (!nodes[referenceNodeId].isActive) return true;
    
    // Если мы еще не знаем координат узла, пропускаем (разрешаем), чтобы не сломать первичный обмен
    if (nodes[referenceNodeId].distance == 0.0f && nodes[referenceNodeId].lat == 0.0f) return true; 

    int senderQ = (int)(nodes[referenceNodeId].azimuth / 90.0f) % 4;
    if (senderQ < 0) senderQ = 0; 
    
    int oppositeQ = (senderQ + 2) % 4;
    return _quadrantNodes[oppositeQ] > 0;
}

void NodeDatabase::updateTopology() {
    uint8_t count = 0;
    float maxD = 0.0f;
    
    for (int i=0; i<4; i++) _quadrantNodes[i] = 0;

    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            count++;
            if (nodes[i].distance > maxD) {
                maxD = nodes[i].distance;
            }
            
            // Заселяем квадранты только узлами, которые достаточно далеко (не толпятся)
            if (nodes[i].distance >= MIN_RELAY_DISTANCE_METERS) {
                int q = (int)(nodes[i].azimuth / 90.0f) % 4;
                if (q >= 0 && q < 4) {
                    _quadrantNodes[q]++;
                }
            }
        }
    }
    
    _activeNodesCount = count;
    _cachedMaxDist = maxD;
    
    LOG_INFO("SYS", "Topology sync: Nodes: %d, MaxDist: %.1fm. Q:[%d,%d,%d,%d]", 
             _activeNodesCount, _cachedMaxDist, _quadrantNodes[0], _quadrantNodes[1], _quadrantNodes[2], _quadrantNodes[3]);
}