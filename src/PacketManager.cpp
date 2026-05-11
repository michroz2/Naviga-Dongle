/**
 * File: PacketManager.cpp
 * Version: 1.38 
 * Изменение: Безопасный парсинг переменной длины для пакетов NodeInfo (Шаг 2).
 * Description: Реализация диспетчера пакетов. Распаковывает Payload и обновляет базу данных.
 */
 #include "PacketManager.h"
 #include "logger.h"
 
 // Инициализация с передачей ссылок на необходимые подсистемы
 PacketManager::PacketManager(NodeDatabase& db, GpsManager& gps, GeoPacker& packer)
     : _nodeDB(db), _gps(gps), _packer(packer), _lastTargetId(0) {
 } // PacketManager::PacketManager()
 
 // Возвращает ID последнего активного узла (используется для вывода на экран "Target")
 uint8_t PacketManager::getLastTargetId() const {
     return _lastTargetId;
 } // PacketManager::getLastTargetId()
 
 // Сброс активной цели (например, если узел был удален)
 void PacketManager::clearLastTargetId() {
     _lastTargetId = 0;
 } // PacketManager::clearLastTargetId()
 
 // Обработчик пакета типа MSG_COORDS
 void PacketManager::handleCoordsPacket(uint8_t senderId, const uint8_t* payload) {
     uint32_t packedCoords;
     memcpy(&packedCoords, payload, sizeof(PayloadCoords));
 
     // Если у нас (локально) нет GPS фикса, мы не можем распаковать дельта-координаты
     if (!_gps.isValid()) {
         LOG_WARN("DISPATCH", "No GPS fix. Saved RAW coords for Node %d", senderId);
         // Просто сохраняем сырые packedCoords в базу. Когда GPS появится, main.cpp их распакует.
         _nodeDB.updateNodeCoords(senderId, 0.0f, 0.0f, packedCoords);
         _lastTargetId = senderId; 
         return;
     } // if (!_gps.isValid())
 
     // Распаковка через GeoPacker на основе нашей локальной позиции
     float unpLat, unpLon;
     _packer.unpack(packedCoords, _gps.getLat(), _gps.getLon(), unpLat, unpLon);
     
     _nodeDB.updateNodeCoords(senderId, unpLat, unpLon, packedCoords);
     _lastTargetId = senderId; 
     LOG_INFO("DISPATCH", "Extracted COORDS: Node %d -> Lat: %.6f, Lon: %.6f", senderId, unpLat, unpLon);
 } // PacketManager::handleCoordsPacket()
 
 // ИЗМЕНЕНИЕ 1.38: Обработчик пакета типа MSG_NODE_INFO с переменной длиной
 void PacketManager::handleNodeInfoPacket(uint8_t senderId, const uint8_t* payload, size_t payloadLen) {
     if (payloadLen < 2) return; // Защита: пакет должен содержать минимум 1 байт роли и 1 байт нуль-терминатора
     
     uint8_t nodeType = payload[0];
     
     char nodeName[24];
     size_t nameLen = payloadLen - 1; // Все байты после роли - это имя
     if (nameLen > 23) nameLen = 23;  // Защита от переполнения (максимум 23 символа)
     
     memcpy(nodeName, payload + 1, nameLen);
     nodeName[nameLen] = '\0'; // Гарантированно закрываем строку
     
     _nodeDB.updateNodeInfo(senderId, nodeName, nodeType);
     LOG_INFO("DISPATCH", "Updated Node %d: Name=%s, Type=%d", senderId, nodeName, nodeType);
 } // PacketManager::handleNodeInfoPacket()
 
 // Обработчик пакета типа MSG_LEAVE
 void PacketManager::handleLeavePacket(uint8_t senderId, const uint8_t* payload) {
     PayloadLeave info;
     memcpy(&info, payload, sizeof(PayloadLeave)); 
     _nodeDB.removeNode(senderId); // Удаляем узел из активных
     LOG_INFO("DISPATCH", "Node %d left the network. Reason code: %d", senderId, info.reason);
 } // PacketManager::handleLeavePacket()
 
 // Главный распределитель пакетов
 void PacketManager::processPacket(const NavigaHeader& header, const uint8_t* payload, size_t payloadLen) {
     // Уверены, что получили валидный пакет. Проверяем и явно добавляем узел в базу, если его там нет.
     if (!_nodeDB.isNodeActive(header.senderId)) {
         _nodeDB.addNode(header.senderId);
     }
 
     // Маршрутизация по типам пакетов
     switch(header.getType()) {
         case MSG_COORDS:
             handleCoordsPacket(header.senderId, payload);
             break;
         case MSG_NODE_INFO:
             handleNodeInfoPacket(header.senderId, payload, payloadLen);
             break;
         case MSG_LEAVE:
             handleLeavePacket(header.senderId, payload);
             break;
         default:
             break;
     } // switch(header.getType())
 } // PacketManager::processPacket()