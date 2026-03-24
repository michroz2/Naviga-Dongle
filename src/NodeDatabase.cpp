/**
 * File: NodeDatabase.cpp
 * Version: 1.2.0
 */
 #include "NodeDatabase.h"
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
         nodes[i].distance = 0.0f; // Дефолт
         nodes[i].azimuth = 0.0f;  // Дефолт
         snprintf(nodes[i].nodeName, sizeof(nodes[i].nodeName), "Node-%d", i);
     }
 }
 
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
         snprintf(nodes[nodeId].nodeName, sizeof(nodes[nodeId].nodeName), "Node-%d", nodeId);
     }
     return &nodes[nodeId];
 }
 
 void NodeDatabase::updateNodeSNR(uint8_t nodeId, float snr) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     const NodeRecord* node = getNode(nodeId); 
     if (node != nullptr) {
         nodes[nodeId].snr = snr;
         nodes[nodeId].lastSeen = millis(); 
     }
 }
 
 // НОВОЕ: Запись геометрии без изменения таймера активности
 void NodeDatabase::updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     const NodeRecord* node = getNode(nodeId);
     if (node != nullptr) {
         nodes[nodeId].distance = dist;
         nodes[nodeId].azimuth = azmt;
     }
 }
 
 void NodeDatabase::updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     getNode(nodeId); 
     nodes[nodeId].lat = lat;
     nodes[nodeId].lon = lon;
     if (packed != 0) {
         nodes[nodeId].packedCoords = packed;
     }
     if (updateTimer) {
         nodes[nodeId].lastSeen = millis();
     }
 }
 
 void NodeDatabase::updateNodeName(uint8_t nodeId, const char* name) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     getNode(nodeId);
     strncpy(nodes[nodeId].nodeName, name, 15);
     nodes[nodeId].nodeName[15] = '\0';
     nodes[nodeId].lastSeen = millis();
 }
 
 void NodeDatabase::removeNode(uint8_t nodeId) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     nodes[nodeId].isActive = false;
 }
 
 void NodeDatabase::cleanup() {
     uint32_t now = millis();
     for (int i = 1; i < MAX_NODES; i++) {
         if (nodes[i].isActive && (now - nodes[i].lastSeen > NODE_TIMEOUT_MS)) {
             nodes[i].isActive = false;
         }
     }
 }
 
 uint8_t NodeDatabase::getActiveNodesCount() const {
     uint8_t count = 0;
     for (int i = 1; i < MAX_NODES; i++) {
         if (nodes[i].isActive) count++;
     }
     return count;
 }