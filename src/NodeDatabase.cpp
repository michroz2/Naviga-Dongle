/**
 * File: NodeDatabase.cpp
 * Version: 1.27 
 * Изменение: Подключение SettingsManager. Функция cleanup теперь использует динамический таймаут из NVS.
 * Description: Реализация локальной базы данных активных узлов в Mesh-сети.
 */
 #include "NodeDatabase.h"
 #include "logger.h"
 #include "configuration.h"
 #include "SettingsManager.h" // ИЗМЕНЕНИЕ 1.27: Подключение менеджера настроек
 #include <string.h>
 
 NodeDatabase::NodeDatabase() {
     _activeNodesCount = 0;
     _cachedMaxDist = 0.0f;
     for (int i=0; i<4; i++) _quadrantNodes[i] = 0;
     
     // Предварительная инициализация массива из 255 узлов (0-й индекс не используется как активный узел)
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
 
 // Получение константного указателя на запись узла (защита от несанкционированного изменения)
 const NodeRecord* NodeDatabase::getNode(uint8_t nodeId) const {
    if (nodeId == 0 || nodeId >= MAX_NODES) return nullptr;
    if (!nodes[nodeId].isActive) return nullptr; 
     return &nodes[nodeId];
 } 

 // Добавление нового узла в базу
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

 // Обновление информации об узле (Role, Name) из пакета NodeInfo
 void NodeDatabase::updateNodeInfo(uint8_t nodeId, const char* name, uint8_t nodeType) {
     if (nodeId == 0 || nodeId >= MAX_NODES) return;
     if (!isNodeActive(nodeId)) addNode(nodeId);
     nodes[nodeId].type = nodeType;
     strncpy(nodes[nodeId].nodeName, name, sizeof(nodes[nodeId].nodeName) - 1);
     nodes[nodeId].nodeName[sizeof(nodes[nodeId].nodeName) - 1] = '\0'; // Гарантированный нуль-терминатор
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
 
 // Обновление координат узла. Параметр updateTimer позволяет обновлять таймер lastSeen 
 // (по умолчанию true, false используется при загрузке слепка из памяти NVS)
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
 
    // Сборщик мусора: деактивирует узлы по таймауту (от которых давно не было вестей)
    void NodeDatabase::cleanup(uint8_t excludeNodeId) {
        uint32_t currentMillis = millis();
        for (int i = 1; i < MAX_NODES; i++) {
            if (i == excludeNodeId) continue; // Защита собственного узла от случайного удаления
            // ИЗМЕНЕНИЕ 1.27: Используем динамический таймаут из NVS (настройки Оператора)
            if (nodes[i].isActive && (currentMillis - nodes[i].lastSeen > settingsManager.settings.nodeActiveTimeoutMs)) {
                nodes[i].isActive = false;
                if (_activeNodesCount > 0) _activeNodesCount--;
                LOG_INFO("SYS", "Node %d removed by timeout", i);
            }
        }
    } 
 
uint8_t NodeDatabase::getActiveNodesCount() const { return _activeNodesCount; }
float NodeDatabase::getCachedMaxDist() const { return _cachedMaxDist; }

// --- ВЕКТОРНЫЙ ФИЛЬТР ---
// Проверяет, есть ли хотя бы один узел в противоположном квадранте относительно узла,
// который переслал нам пакет (referenceNodeId). Если нет, значит мы — географический тупик сети.
bool NodeDatabase::hasNodesInOppositeDirection(uint8_t referenceNodeId) const {
    if (referenceNodeId == 0 || referenceNodeId >= MAX_NODES) return true;
    if (!nodes[referenceNodeId].isActive) return true;
    
    // Если мы не знаем дистанции до отправителя, разрешаем ретрансляцию (Fallback)
    if (nodes[referenceNodeId].distance == 0.0f && nodes[referenceNodeId].lat == 0.0f) return true; 

    // Определяем в каком мы квадранте (0, 1, 2, 3) находится отправитель относительно нас
    int senderQ = (int)(nodes[referenceNodeId].azimuth / 90.0f) % 4;
    if (senderQ < 0) senderQ = 0; 
    
    // Вычисляем противоположный квадрант
    int oppositeQ = (senderQ + 2) % 4;
    
    // Проверяем, есть ли узлы в противоположном квадранте (данные пересчитываются в updateTopology)
    return _quadrantNodes[oppositeQ] > 0;
}

// Тяжеловесная функция пересчета сетевой топологии
void NodeDatabase::updateTopology() {
    uint8_t count = 0;
    float maxD = 0.0f;
    
    // Сбрасываем счетчики квадрантов
    for (int i=0; i<4; i++) _quadrantNodes[i] = 0;

    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            count++;
            // Поиск максимальной дистанции в группе для фильтра "Компактная группа"
            if (nodes[i].distance > maxD) {
                maxD = nodes[i].distance;
            }
            
            // Распределение узлов по квадрантам для векторного фильтра
            // Исключаем узлы, которые находятся слишком близко (< 20м), 
            // так как ошибка азимута на коротких дистанциях слишком велика
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

// Искусственное состаривание таймеров lastSeen.
// Используется при включении (Warm Start), когда мы загрузили узлы из флеш памяти, 
// но еще не получили от них свежих подтверждений в эфире. Узлы становятся "серыми".
void NodeDatabase::ageAllNodes(uint32_t ageMs) {
    uint32_t currentMillis = millis();
    for (int i = 1; i < MAX_NODES; i++) {
        if (nodes[i].isActive) {
            // Защита от математического переполнения (overflow)
            if (currentMillis >= ageMs) {
                nodes[i].lastSeen = currentMillis - ageMs;
            } else {
                nodes[i].lastSeen = 0;
            }
        }
    }
} //NodeDatabase.cpp