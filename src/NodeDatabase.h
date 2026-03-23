/**
 * File: NodeDatabase.h
 * Version: 1.1.0
 * Description: Добавлено хранение индивидуального SNR для каждого узла.
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
     float snr; // НОВОЕ: Сырое значение уровня сигнала
 };
 
 class NodeDatabase {
 public:
     NodeDatabase();
 
     const NodeRecord* getNode(uint8_t nodeId);
     void updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer = true);
     void updateNodeName(uint8_t nodeId, const char* name);
     
     // НОВОЕ: Метод для обновления физического качества связи
     void updateNodeSNR(uint8_t nodeId, float snr);
 
     void removeNode(uint8_t nodeId);
     void cleanup();
     uint8_t getActiveNodesCount() const;
 
 private:
     NodeRecord nodes[MAX_NODES];
 };
 
 #endif // NODE_DATABASE_H