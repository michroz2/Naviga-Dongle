/**
 * File: NodeDatabase.h
 * Version: 1.17 Изменение: Добавлен метод hasNodesInOppositeDirection и массив квадрантов.
 * Description: Заголовочный файл базы данных узлов.
 */
 #ifndef NODE_DATABASE_H
 #define NODE_DATABASE_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 
 #define MAX_NODES 255

 // Таймаут неактивности узла: 3 часа (3 * 60 * 60 * 1000 = 10800000 мс)
 #define NODE_TIMEOUT_MS 10800000 
 
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
 }; // struct NodeRecordхзщ98
 
 class NodeDatabase {
    public:
        NodeDatabase();
    
    const NodeRecord* getNode(uint8_t nodeId) const;

     bool isNodeActive(uint8_t nodeId) const;       
    
     void addNode(uint8_t nodeId);                  
    
     void updateNodeCoords(uint8_t nodeId, float lat, float lon, uint32_t packed, bool updateTimer = true);
     void updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType);
     void updateNodeSNR(uint8_t nodeId, float snr);
     void updateNodeDistanceAzimuth(uint8_t nodeId, float dist, float azmt);
 
     void removeNode(uint8_t nodeId);
     void cleanup(uint8_t excludeNodeId = 0); 
     
     uint8_t getActiveNodesCount() const;
     
    // Методы топологии
    void updateTopology();
    float getCachedMaxDist() const;
    
    // НОВОЕ: Векторный фильтр
    bool hasNodesInOppositeDirection(uint8_t referenceNodeId) const;

    // Искусственное старение узлов (сдвиг lastSeen в прошлое) для Warm Start
    void ageAllNodes(uint32_t ageMs);

 private:
     NodeRecord nodes[MAX_NODES];

     // Кэшированные значения топологии
    uint8_t _activeNodesCount;
    float _cachedMaxDist;
    uint8_t _quadrantNodes[4]; // 0: 0-90, 1: 90-180, 2: 180-270, 3: 270-360
    
 }; // class NodeDatabase
 
 #endif // NODE_DATABASE_H