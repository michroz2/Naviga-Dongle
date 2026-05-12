/**
 * File: CoordProcessor.h
 * Version: 1.00 (Refactored from main.cpp v1.44)
 * Description: Выделенный модуль для гео-математики: расчет дистанций, азимутов и распаковка координат.
 */

 #ifndef COORD_PROCESSOR_H
 #define COORD_PROCESSOR_H
 
 #include <Arduino.h>
 #include "NodeDatabase.h"
 #include "GpsManager.h"
 #include "GeoPacker.h"
 
 class CoordProcessor {
 public:
     CoordProcessor();
 
     /**
      * Основной цикл обработки координат всех узлов в базе.
      * @param nodeDB Ссылка на базу узлов
      * @param gps Ссылка на менеджер GPS донгла
      * @param packer Ссылка на упаковщик координат
      * @param isFastTracker Флаг оптимизации (пропуск расчетов при движении)
      */
     void process(NodeDatabase& nodeDB, GpsManager& gps, GeoPacker& packer, bool isFastTracker);
 
 private:
     bool _isLonScaleSet;
     float _lastScaleLat;
 };
 
 #endif // COORD_PROCESSOR_H