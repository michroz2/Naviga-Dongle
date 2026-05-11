/**
 * File: NodeDatabase.h
 * Version: 1.38 
 * Изменение: Увеличен размер буфера nodeName до 24 байт (Шаг 2).
 * Description: Заголовочный файл локальной базы данных активных узлов.
 */
 #ifndef NODE_DATABASE_H
 #define NODE_DATABASE_H
 
 #include <Arduino.h>
 #include "NavigaProtocol.h"
 
 #define MAX_NODES 255

 // Старый дефолтный таймаут неактивности узла (заменен на динамический в SettingsManager, но оставлен для резерва)
 #define NODE_TIMEOUT_MS 10800000 
 
 // Структура хранения информации об одном активном узле
 struct NodeRecord {
     uint8_t nodeId;
     uint8_t type;       // Тип узла (Роль)
     char nodeName[24];  // ИЗМЕНЕНИЕ 1.38: Увеличен до 24 байт
     float lat;
     float lon;
     uint32_t packedCoords; // Сырые запакованные координаты (до распаковки GeoPacker)
     uint32_t lastSeen;     // Время (millis()) последнего приема пакета от узла
     bool isActive;
     float snr;             // Последний измеренный уровень сигнала от узла
     float distance;        // Рассчитанная дистанция в метрах до локального устройства
     float azimuth;         // Рассчитанный азимут (направление) от локального устройства
 }; // struct NodeRecord
 
 class NodeDatabase {
    public:
        NodeDatabase();
    
    // Получение указателя на константную запись
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
    
    // Векторный фильтр
    bool hasNodesInOppositeDirection(uint8_t referenceNodeId) const;

    // Искусственное старение узлов (сдвиг lastSeen в прошлое) для Warm Start
    void ageAllNodes(uint32_t ageMs);

 private:
     NodeRecord nodes[MAX_NODES]; // Статический массив на 255 узлов

     // Кэшированные значения топологии
    uint8_t _activeNodesCount;
    float _cachedMaxDist;
    uint8_t _quadrantNodes[4]; // 0: 0-90, 1: 90-180, 2: 180-270, 3: 270-360 градусов
    
 }; // class NodeDatabase
 
 #endif // NODE_DATABASE_H