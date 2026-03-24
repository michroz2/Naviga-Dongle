/**
 * File: NodeDatabase.h
 * Version: 1.3.1
 * Description: Заголовочный файл базы данных узлов.
 * Изменение: Приведение к новым правилам оформления (комментирование скобок).
 */
 #ifndef NODE_DATABASE_H
 #define NODE_DATABASE_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 
 #define MAX_NODES 255
 #define NODE_TIMEOUT_MS 300000 
 
 struct NodeRecord {
     uint8_t nodeId;
     uint8_t type;       // Тип узла
     char nodeName[12];  // Соразмерно Payload (с запасом под \0)
     float lat;
     float lon;
     uint32_t packedCoords;
     uint32_t lastSeen;
     bool isActive;
     float snr;
     float distance; 
     float azimuth;  
 }; // struct NodeRecord
 
 class NodeDatabase {
 public:
     NodeDatabase();
 
     const NodeRecord* getNode(uint8_t nodeId);
     void updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer = true);
     void updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType);
     void updateNodeSNR(uint8_t nodeId, float snr);
     void updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt);
 
     void removeNode(uint8_t nodeId);
     void cleanup();
     uint8_t getActiveNodesCount() const;
 
 private:
     NodeRecord nodes[MAX_NODES];
 }; // class NodeDatabase
 
 #endif // NODE_DATABASE_H