/**
 * File: NodeDatabase.h
 * Version: 1.2.9
 * Description: Заголовочный файл локальной базы данных узлов (соседей).
 * Поддерживает хранение сырых координат для отложенной распаковки.
 */
 #ifndef NODE_DATABASE_H
 #define NODE_DATABASE_H
 
 #include <Arduino.h>
 
 const uint16_t MAX_NODES = 256; 
 const uint32_t NODE_TIMEOUT_MS = 300000; // Время "жизни" узла без связи (5 минут)
 
 // Структура данных об одном соседе
 struct NodeRecord {
     uint8_t nodeId;
     char name[13];       // 12 символов + нуль-терминатор
     float lat;
     float lon;
     uint32_t packedCoords; // НОВОЕ: Сырые сжатые координаты
     uint32_t lastSeen;   // Время последнего приема пакета
     bool isActive;       // Флаг присутствия узла в сети
 }; // struct NodeRecord
 
 class NodeDatabase {
 public:
     NodeDatabase();
 
     // Обновление координат. Добавлен параметр packedCoords и флаг обновления времени.
     void updateNodeCoords(uint8_t id, float lat, float lon, uint32_t packedCoords = 0, bool updateTime = true);
     
     // Обновление имени узла
     void updateNodeName(uint8_t id, const char* name);
     
     // Удаление узла из базы
     void removeNode(uint8_t id);
     
     // Очистка устаревших узлов
     void cleanup();
     
     // Получить количество активных узлов
     uint8_t getActiveNodesCount() const;
 
     // Получить указатель на данные узла
     const NodeRecord* getNode(uint8_t id) const;
 
 private:
     NodeRecord nodes[MAX_NODES];
 
 }; // class NodeDatabase
 
 #endif // NODE_DATABASE_H