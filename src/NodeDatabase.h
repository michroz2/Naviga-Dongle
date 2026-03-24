/**
 * File: NodeDatabase.h
 * Version: 1.2.0
 * Description: Добавлено кэширование дистанции и азимута для каждого узла.
 */
 #ifndef NODE_DATABASE_H
 #define NODE_DATABASE_H
 
 #include <Arduino.h>
 
 #define MAX_NODES 255
 #define NODE_TIMEOUT_MS 300000 
 
 struct NodeRecord {
     uint8_t nodeId;
     char nodeName[16];
     float lat;
     float lon;
     uint32_t packedCoords;
     uint32_t lastSeen;
     bool isActive;
     float snr;
     float distance; // НОВОЕ: Дистанция от нас до узла (в метрах)
     float azimuth;  // НОВОЕ: Азимут от нас на узел (в градусах)
 };
 
 class NodeDatabase {
 public:
     NodeDatabase();
 
     const NodeRecord* getNode(uint8_t nodeId);
     void updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer = true);
     void updateNodeName(uint8_t nodeId, const char* name);
     void updateNodeSNR(uint8_t nodeId, float snr);
     
     // НОВОЕ: Метод для обновления локальной геометрии
     void updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt);
 
     void removeNode(uint8_t nodeId);
     void cleanup();
     uint8_t getActiveNodesCount() const;
 
 private:
     NodeRecord nodes[MAX_NODES];
 };
 
 #endif // NODE_DATABASE_H