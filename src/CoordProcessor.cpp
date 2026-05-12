/**
 * File: CoordProcessor.cpp
 * Version: 1.00 (Refactored from main.cpp v1.44)
 * Description: Реализация фоновых гео-вычислений. Код перенесен из main.cpp без изменений.
 */

 #include "CoordProcessor.h"
 #include "logger.h"
 
 CoordProcessor::CoordProcessor() : _isLonScaleSet(false), _lastScaleLat(0.0f) {
 }
 
 void CoordProcessor::process(NodeDatabase& nodeDB, GpsManager& gps, GeoPacker& packer, bool isFastTracker) {
     if (!gps.isValid()) return;
 
     float currentLat = gps.getLat();
     float currentLon = gps.getLon();
 
     // Динамический пересчет масштаба при смещении по широте (> 1 градуса)
     if (!_isLonScaleSet || abs(currentLat - _lastScaleLat) > 1.0f) {
         packer.updateLonScale(currentLat); // Устанавливаем масштаб по широте
         _lastScaleLat = currentLat;
         _isLonScaleSet = true;
         LOG_INFO("SYS", "Longitude scale updated for Lat: %.4f", currentLat);
     } 
 
     // Если мы не в режиме быстрого трекера, обновляем топологию всех активных узлов
     if (!isFastTracker) {
         for (int i = 1; i < 255; i++) {
             const NodeRecord* node = nodeDB.getNode(i);
             
             // Обрабатываем только активные узлы (кроме самого себя, хотя myNodeId отфильтруется логикой расстояния 0)
             if (node != nullptr && node->isActive) {
                 
                 // 1. Распаковка координат: если у нас есть сырые данные, но нет float значений
                 if (node->packedCoords != 0 && node->lat == 0.0f && node->lon == 0.0f) {
                     float unpLat, unpLon;
                     packer.unpack(node->packedCoords, currentLat, currentLon, unpLat, unpLon);
                     
                     // Обновляем координаты в базе без изменения флага активности
                     nodeDB.updateNodeCoords(i, unpLat, unpLon, node->packedCoords, false);
                 } 
                 
                 // 2. Расчет относительной топологии (Дистанция и Азимут)
                 if (node->lat != 0.0f || node->lon != 0.0f) {
                     float d = gps.distanceTo(node->lat, node->lon);
                     float a = gps.courseTo(node->lat, node->lon);
                     
                     nodeDB.updateNodeDistanceAzimuth(i, d, a);
                 } 
             } 
         } 
     }
 } //CoordProcessor.cpp