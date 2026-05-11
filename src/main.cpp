/**
 * Project: Naviga-Dongle (T-Beam v1.1 / T-Energy S3 + Custom E22 + GPS)
 * File: main.cpp
 * Version: 1.39 
 * Изменение: Команда requestReset теперь вызывает очистку флэш-памяти (UC-08 Factory Reset).
 * Description: Главный файл оркестратора.
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <SPI.h>              
 #include "configuration.h"
 #include "logger.h"            
 #include "GeoPacker.h"        
 #include "NavigaProtocol.h"
 #include "NodeDatabase.h"     
 #include "Retranslation.h"    
 #include "GpsManager.h"        
 #include "DisplayManager.h"   
 #include "RadioManager.h"     
 #include "PowerManager.h"     
 #include "PacketManager.h"    
 #include "TxManager.h"        
 #include "BleManager.h"       
 #include "SettingsManager.h"  
 
 // --- НАСТРОЙКИ СКАНИРОВАНИЯ ---
 uint32_t networkScanDuration = 30000; // 30 секунд сканирования при включении
 
 uint8_t myNodeId = 0;   // Локальный ID устройства в Mesh-сети
 uint8_t myMsgSeq = 0;   // Счетчик исходящих пакетов (sequence)
 uint8_t myNodeType = NODE_RELAY; // Роль узла по умолчанию (перезапишется из настроек)
 
 // Инициализация глобальных менеджеров подсистем
 PowerManager power;                                       
 DisplayManager display(0x3c, I2C_SDA, I2C_SCL); 
 GpsManager gps;                                        
 RadioManager radio; 
 GeoPacker packer;                                       
 NodeDatabase nodeDB;                                   
 Retranslation router;                                   
 PacketManager packetManager(nodeDB, gps, packer);
 TxManager txManager(radio, packer, myNodeId, myMsgSeq);
 BleManager bleManager;                                 
 
 volatile bool receivedFlag = false; // Флаг прерывания от модуля LoRa (ISR)                   
 
 // ISR Коллбэк для обработки прерывания (Packet Received)
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR // Помещаем функцию в оперативную память для быстродействия
 #endif
 void setFlag(void) {
     receivedFlag = true;
 } 
 
 // Системные таймеры
 uint32_t lastTxTime = 0;                               
 uint32_t lastGpsLogTime = 0;                            
 uint32_t lastCleanupTime = 0;                               
 uint32_t lastHeartbeatTime = 0;   
 uint32_t lastTopologyUpdateTime = 0;
 uint32_t lastTelemetryTime = 0; // Таймер для телеметрии
 
 bool isLonScaleSet = false; // Флаг: был ли рассчитан коэффициент сжатия долготы
 float lastScaleLat = 0.0f;  // Широта, при которой последний раз был рассчитан масштаб                            
 
 // Расчет джиттера (рандомизированной задержки) для умной ретрансляции пакета
 uint32_t calculateRelayJitter(uint8_t myRole, uint8_t senderRole, float snr) {
     uint32_t minMs, maxMs;
     
     // Определяем базовые окна джиттера в зависимости от нашей роли
     if (myRole == NODE_RELAY) {
         minMs = RELAY_JITTER_MIN_MS;
         maxMs = RELAY_JITTER_MAX_MS;
     } else {
         minMs = STALKER_JITTER_MIN_MS;
         maxMs = STALKER_JITTER_MAX_MS;
     }
 
     // VIP-Маршрутизация: Если мы ретранслируем пакет ТРЕКЕРА, даем ему зеленый свет
     if (senderRole == NODE_TRACKER) {
         maxMs /= 2; // Ускоряем в 2 раза
         if (maxMs < minMs) maxMs = minMs; 
     }
 
     // Мапим задержку на основании качества сигнала (SNR)
     // Чем лучше сигнал, тем БОЛЬШЕ задержка (передает дальний узел)
     long snrInt = constrain((long)snr, -15, 5);
     uint32_t baseDelay = map(snrInt, -15, 5, minMs, maxMs);
     
     return baseDelay + random(0, 50); // Добавляем небольшую случайность для разрешения коллизий
 }
 
 // Расчет показателя "качества связи" для вывода на экран (от 1 до 10)
 int getConnectionQuality(uint8_t targetId) {
     if (targetId == myNodeId) return 10; 
 
     const NodeRecord* target = nodeDB.getNode(targetId);
     if (target == nullptr || targetId == 0) return 0;
     
     // Если от узла давно не было вестей, качество 0
     if (millis() - target->lastSeen > settingsManager.settings.nodeConnectionTimeout) return 0;
     
     if (target->snr <= -99.0f) return 0; 
 
     // Мапим физический SNR в читаемый балл (1-10)
     int q = map((long)target->snr, -11, 5, 1, 10);
     if (q < 1) q = 1;
     if (q > 10) q = 10;
     return q;
 } 
 
 // Коллбэк для обновления экрана (используется внутри менеджеров)
 void updateScreenCb(String line1, String line2, String line3, String line4) {
     display.showStatus(line1, line2, line3, line4);
 } 
 
 // Коллбэк сброса питания GPS
 void cycleGpsPowerCb() {
     power.cycleGpsPower();
 } 
 
 // Обработка коллизии: если два узла заняли один ID
 void handleCollision() {
    uint8_t oldId = myNodeId;
    nodeDB.removeNode(oldId); 
    
    // Генерируем новый уникальный ID
    randomSeed(esp_random());
    do {
        myNodeId = random(1, 255);
    } while (nodeDB.getNode(myNodeId) != nullptr); // Проверяем, свободен ли он в базе
    
    nodeDB.addNode(myNodeId); 
    
    LOG_WARN("COLLISION", "ID %d is taken! Switched to new ID: %d", oldId, myNodeId);
    
    myMsgSeq = 0; // Сбрасываем счетчик пакетов
    
    char myName[24]; // Буфер расширен до 24 байт
    snprintf(myName, sizeof(myName), "Node-%d", myNodeId);
    
    // Рассылаем новый ID по сети с наивысшим приоритетом
    txManager.sendNodeInfo(myName, myNodeType, TX_CRITICAL);
    nodeDB.updateNodeInfo(myNodeId, myName, myNodeType); 
 
    // Сохраняем изменения в энергонезависимую память (NVS)
    settingsManager.settings.nodeId = myNodeId;
    settingsManager.save();
    settingsManager.saveNodesSnapshot(nodeDB);
 } 
 
// Универсальная функция первоначального сканирования и немого периода (Warm/Cold Start)
void scanNetwork(bool isWarmStart) {
    if (isWarmStart) {
        LOG_INFO("SYS", "Warm Start: Silent listening for %d ms...", networkScanDuration);
    } else {
        LOG_INFO("SYS", "Cold Start: Scanning for %d ms...", networkScanDuration);
    }
    
    display.toggleLed();
    
    uint32_t scanStart = millis();
    uint32_t lastDispUpdate = 0;
    
    receivedFlag = false;
    radio.startReceive(); 
    
    // Цикл немого прослушивания радиоэфира
    while (millis() - scanStart < networkScanDuration) {
        uint32_t now = millis();
        
        // Обновляем дисплей каждую секунду
        if (now - lastDispUpdate >= 1000) {
            lastDispUpdate = now;
            uint32_t left = (networkScanDuration - (now - scanStart)) / 1000;
            
            String startTitle = "Start-" + String(bleManager.macSuffix);
            
            // Получаем общее количество узлов и вычитаем себя (Донгл)
            uint8_t totalNodes = nodeDB.getActiveNodesCount();
            uint8_t foundNeighbors = (totalNodes > 0) ? (totalNodes - 1) : 0;
            
            display.showStatus(startTitle, 
                               "Time left: " + String(left) + " s", 
                               "Neighbors: " + String(foundNeighbors), 
                               "Please wait...");
        } 

        // Обработка входящих пакетов во время сканирования
        if (receivedFlag) {
            noInterrupts(); receivedFlag = false; interrupts();
            
            size_t len = radio.getPacketLength();
            if (len >= sizeof(NavigaHeader)) {
                uint8_t rxBuffer[256];             
                int state = radio.readData(rxBuffer, len); 
                if (state == RADIOLIB_ERR_NONE) {
                    NavigaHeader rxHeader;
                    memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
                    size_t payloadLen = len - sizeof(NavigaHeader);
                    
                    if (router.isValidPacket(rxHeader.getType(), payloadLen)) {
                        if (!router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                            packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen);
                        } 
                    } 
                } 
            } 
            radio.startReceive();
        } 
        
        gps.update(); // Поддерживаем опрос GPS
    } 
    
    display.toggleLed();
    
    // Если это Cold Start (или у нас почему-то нет ID), генерируем новый
    if (!isWarmStart || myNodeId == 0) {
        randomSeed(esp_random());
        do {
            myNodeId = random(1, 255);
        } while (nodeDB.getNode(myNodeId) != nullptr); 
        
        nodeDB.addNode(myNodeId); 
        LOG_INFO("SYS", "Scan complete. Selected unique Node ID: %d", myNodeId);
    } else {
        LOG_INFO("SYS", "Silent listening complete. Kept Node ID: %d", myNodeId);
    }
    
    display.showStatus("Scan Complete", "My ID:", String(myNodeId), "Starting...");
    delay(2000);
}
 
 // === НАЧАЛЬНАЯ НАСТРОЙКА ===
 void setup() {
     delay(500); 
     Serial.begin(115200);
     unsigned long start = millis();
     while (!Serial && (millis() - start < 3000)); 
     LOG_INFO("SYS", "--- DONGLE BOOT START ---");
     
     // Отключаем встроенный LoRa, если это плата T-Beam v1.1
     #ifdef BOARD_T_BEAM_V11
     pinMode(LORA_ONBOARD_CS, OUTPUT);
     digitalWrite(LORA_ONBOARD_CS, HIGH);
     #endif
 
     // Аппаратный сброс LoRa-модуля
     pinMode(LORA_RST, OUTPUT);
     digitalWrite(LORA_RST, LOW);
     delay(20);  
     digitalWrite(LORA_RST, HIGH);
     delay(50);  
 
     Wire.begin(I2C_SDA, I2C_SCL);                       
     SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
     
     power.init();
     display.init(); 
     display.showLogo();
     gps.init(updateScreenCb, cycleGpsPowerCb); 
 
     bleManager.init();
     
     settingsManager.init(); // Инициализация энергонезависимой памяти
     myNodeId = settingsManager.settings.nodeId;
     myNodeType = settingsManager.settings.nodeType;
 
     settingsManager.loadNodesSnapshot(nodeDB); // Загружаем "снимки" соседей
 
     // Искусственно стартим восстановленные узлы, чтобы при Warm Start они были "серыми"
     nodeDB.ageAllNodes(settingsManager.settings.nodeConnectionTimeout + 1000);
 
     // Гарантируем, что наш локальный узел свежий и активный поверх слепка
     char myName[24]; // Буфер расширен до 24 байт
     strncpy(myName, settingsManager.settings.nodeName, sizeof(myName)-1);
     myName[sizeof(myName)-1] = '\0';
     
     nodeDB.addNode(myNodeId);
     nodeDB.updateNodeInfo(myNodeId, myName, myNodeType);
 
     // Если устройство является стационарным ретранслятором, задаем ему статические координаты
     if (myNodeType == NODE_RELAY) {
         gps.setStaticLocation(RELAY_STATIC_LAT, RELAY_STATIC_LON);
     }
 
     display.showStatus("System Init...", "GPS Init Done", "Init LoRa...", "");
     if (!radio.init(setFlag)) {
         display.showStatus("ERROR", "LoRa Init Failed", "Check Logs", "");
         delay(3000);
     } 
 
     if (myNodeId == 0 || !settingsManager.settings.isConfigured) {
         scanNetwork(false); // Cold Start - выбор свободного ID
         settingsManager.settings.nodeId = myNodeId;
         settingsManager.settings.isConfigured = true;
         settingsManager.save();
     } else {
         scanNetwork(true);  // Warm Start - просто слушаем эфир 30 сек
     }
     
     // Рассылаем по эфиру свое приветствие
     txManager.sendNodeInfo(myName, myNodeType, TX_NORMAL);
     
     lastTxTime = millis(); 
 } 
 
 // === ГЛАВНЫЙ ЦИКЛ ОРКЕСТРАТОРА ===
 void loop() {
     uint32_t currentMillis = millis();
     float currentSpeed = gps.getSpeed();
 
    // ==========================================================
    // ОБРАБОТКА КОМАНД ОТ СМАРТФОНА ПО BLUETOOTH
    // ==========================================================
    
    // Обработка запросов настроек (UC-04 Pairing)
    if (bleManager.requestIdentitySync) {
        bleManager.requestIdentitySync = false;
        bleManager.sendIdentity(
            settingsManager.settings.nodeId, 
            settingsManager.settings.nodeName, 
            settingsManager.settings.nodeType
        );
        LOG_INFO("BLE", "Sent Identity config to Smartphone");
    }

    if (bleManager.requestSysConfigSync) {
        bleManager.requestSysConfigSync = false;
        bleManager.sendSysConfig(
            settingsManager.settings.txIntervalMoving,
            settingsManager.settings.txIntervalStill,
            settingsManager.settings.nodeConnectionTimeout,
            settingsManager.settings.nodeActiveTimeoutMs
        );
        LOG_INFO("BLE", "Sent System Config to Smartphone");
    }
    
    // Синхронизация всей топологии узлов в смартфон (по запросу)
    if (bleManager.requestFullSync) {
         bleManager.requestFullSync = false;
         for (int i = 1; i < 255; i++) {
             const NodeRecord* node = nodeDB.getNode(i);
             if (node != nullptr && node->isActive) {
                 BleEvtNodeUpdate update;
                 update.opCode = EVT_NODE_UPDATE;
                 update.nodeId = node->nodeId;
                 update.nodeRole = node->type;
                 strncpy(update.nodeName, node->nodeName, sizeof(update.nodeName)-1);
                 update.nodeName[sizeof(update.nodeName)-1] = '\0';
                 update.lat = node->lat;
                 update.lon = node->lon;
                 update.distance = node->distance;
                 update.azimuth = node->azimuth;
                 update.snr = node->snr;
                 update.lastSeenAge = millis() - node->lastSeen;
                 
                 bleManager.sendNodeUpdate(update);
                 delay(5); // Небольшая задержка, чтобы не переполнить MTU стек Bluetooth
             }
         }
         LOG_INFO("BLE", "Full topology sync sent to Smartphone");
     }
 
     // Получены новые настройки идентификации со смартфона
     if (bleManager.hasNewIdentity) {
         bleManager.hasNewIdentity = false;
         myNodeId = bleManager.newIdentity.myNodeId;
         myNodeType = bleManager.newIdentity.myRole;
         
         settingsManager.settings.nodeId = myNodeId;
         settingsManager.settings.nodeType = myNodeType;
         strncpy(settingsManager.settings.nodeName, bleManager.newIdentity.myName, sizeof(settingsManager.settings.nodeName)-1);
         settingsManager.settings.nodeName[sizeof(settingsManager.settings.nodeName)-1] = '\0';
         settingsManager.save();
 
         // Уведомляем Mesh-сеть о смене нашего Имени/Роли
         txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType, TX_CRITICAL);
         nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName, myNodeType);
         
         settingsManager.saveNodesSnapshot(nodeDB);
         LOG_INFO("BLE", "Identity updated from App and saved (ID: %d)", myNodeId);
     }
 
     // Получены новые системные тайминги со смартфона
     if (bleManager.hasNewSysConfig) {
        bleManager.hasNewSysConfig = false;
        settingsManager.settings.txIntervalMoving = bleManager.newSysConfig.txIntervalMoving;
        settingsManager.settings.txIntervalStill = bleManager.newSysConfig.txIntervalStill;
        
        settingsManager.settings.nodeConnectionTimeout = bleManager.newSysConfig.nodeConnectionTimeout;
        settingsManager.settings.nodeActiveTimeoutMs = bleManager.newSysConfig.nodeActiveTimeoutMs;
        
        settingsManager.save();
        LOG_INFO("BLE", "SysConfig updated from App and saved");
    }

     // Принудительная очистка базы соседей
     if (bleManager.requestClearDB) {
         bleManager.requestClearDB = false;
         for (int i = 1; i < 255; i++) {
             if (i != myNodeId) {
                 nodeDB.removeNode(i);
             }
         }
         settingsManager.saveNodesSnapshot(nodeDB); 
         LOG_INFO("BLE", "Node database cleared via App command");
     }
 
     // ИЗМЕНЕНИЕ 1.39: Аппаратный сброс (перезагрузка) с полной очисткой флэш-памяти (Factory Reset)
     if (bleManager.requestReset) {
         LOG_INFO("BLE", "Executing FACTORY RESET via App Command...");
         settingsManager.factoryReset(); // Стираем все данные из NVS (настройки и слепки узлов)
         delay(500);
         ESP.restart(); // Перезагружаем контроллер, чтобы начать с Cold Start
     }
     // ==========================================================
 
    // Определяем, является ли Трекер "бегущим" (Скорость > порога)
    bool isFastTracker = (myNodeType == NODE_TRACKER && currentSpeed > TRACKER_FAST_SPEED_KMPH);
 
    // Периодический пересчет топологии сети (Азимуты, Квадранты, Дистанции)
    if (gps.isValid() && (currentMillis - lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS)) {
        if (isFastTracker) {
            // Оптимизация процессора: Бегущий трекер не тратит ресурсы на топологию
            LOG_INFO("SYS", "Topology sync skipped: Tracker is running.");
        } else {
            nodeDB.updateTopology();
        }
        lastTopologyUpdateTime = currentMillis;
    }
 
    // Периодическая очистка устаревших узлов (Garbage Collector)
    if (currentMillis - lastCleanupTime > CLEANUP_INTERVAL_MS) {
        nodeDB.cleanup(myNodeId); 
        lastCleanupTime = currentMillis;
    } 
 
    // Регулярная отправка "живого" пинга (NodeInfo) - Heartbeat
    if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
        if (myNodeId != 0) { 
            txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType, TX_NORMAL);
            nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName, myNodeType); 
            LOG_INFO("ACTION", "Heartbeat sent: NodeInfo");
        } 
        lastHeartbeatTime = currentMillis;
    } 
 
     // Обновляем данные с GPS-чипа
     gps.update();

     // Регулярная отправка Телеметрии по Bluetooth
     if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
         lastTelemetryTime = currentMillis;
         if (bleManager.getBleStatus() == BLE_CONNECTED) {
             uint8_t gpsValid = gps.isValid() ? 1 : 0;
             uint8_t sats = (uint8_t)gps.getSatellites();
             uint8_t battPct = power.getBatteryPercent();
             uint16_t battV = power.getBatteryVoltage();
             bleManager.sendMyStatus(gpsValid, sats, battPct, battV);
         }
     }
 
     // === ОБРАБОТКА ВХОДЯЩИХ ПАКЕТОВ LORA (RX) ===
     if (receivedFlag) {
         noInterrupts(); receivedFlag = false; interrupts(); // Сброс ISR флага
         
         size_t len = radio.getPacketLength();
         if (len > 0) {
             uint8_t rxBuffer[256];             
             int state = radio.readData(rxBuffer, len); 
             if (state == RADIOLIB_ERR_NONE) {
                 
                 float currentSNR = radio.getSNR();
                 
                 // Если пакет прошел базовую проверку на размер
                 if (len >= sizeof(NavigaHeader)) {
                     NavigaHeader rxHeader;
                     memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader)); // Извлекаем заголовок
                     size_t payloadLen = len - sizeof(NavigaHeader);
                     
                     bool isCollision = false;
                     bool isOwnEcho = false;
 
                     // Проверка на коллизии (конфликт ID)
                     if (rxHeader.relayId == myNodeId) {
                         isCollision = true; // Кто-то ретранслирует под нашим ID
                     } else if (rxHeader.senderId == myNodeId) {
                         // Кто-то отправляет оригинальный пакет с нашим ID. Нужно проверить, не наше ли это эхо.
                         int8_t seqDiff = (int8_t)(myMsgSeq - rxHeader.msgSeq);
                         if (seqDiff <= 0 || seqDiff > 10) {
                             isCollision = true; // Коллизия!
                         } else {
                             isOwnEcho = true;   // Наше собственное эхо, перехваченное от ретранслятора
                         } 
                     } 
 
                     if (isCollision) {
                         handleCollision();
                     } 
 
                     // Обновляем показатель SNR для узла, который физически передал нам пакет (relayId)
                     if (rxHeader.relayId != myNodeId) {
                         nodeDB.updateNodeSNR(rxHeader.relayId, currentSNR);
                     } 
 
                     if (isOwnEcho) {
                         // Эхо игнорируем
                     } else if (!router.isValidPacket(rxHeader.getType(), payloadLen)) {
                         LOG_WARN("LORA", "Invalid packet format/size! Type: %d, Len: %d", rxHeader.getType(), payloadLen);
                     } else if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
                         // --- IMPLICIT ACK (Подавление перехватом) ---
                         // Если мы услышали пакет, который уже знаем (дубликат), 
                         // мы проверяем векторный фильтр. Если мы тупик, мы отменяем свою задачу ретрансляции
                         if (!nodeDB.hasNodesInOppositeDirection(rxHeader.relayId)) {
                             txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
                         } 
                         
                     } else {
                         // --- ПАКЕТ УСПЕШНО ПРОШЕЛ ВСЕ ФИЛЬТРЫ ---
                         LOG_INFO("LORA", "Valid pkt Type %d from Node %d (Relay: %d, Seq: %d, SNR: %.1f)", 
                                  rxHeader.getType(), rxHeader.senderId, rxHeader.relayId, rxHeader.msgSeq, currentSNR);
                         
                            // Флаг для определения, впервые ли мы слышим этот узел
                            bool isNewNode = !nodeDB.isNodeActive(rxHeader.senderId);
                            // Распаковка payload в зависимости от типа
                            packetManager.processPacket(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen);
 
                            // Уведомление смартфона о новых данных узла по BLE
                            const NodeRecord* updatedNode = nodeDB.getNode(rxHeader.senderId);
                            if (updatedNode != nullptr) {
                                BleEvtNodeUpdate update;
                                update.opCode = EVT_NODE_UPDATE;
                                update.nodeId = updatedNode->nodeId;
                                update.nodeRole = updatedNode->type;
                                strncpy(update.nodeName, updatedNode->nodeName, sizeof(update.nodeName)-1);
                                update.nodeName[sizeof(update.nodeName)-1] = '\0';
                                update.lat = updatedNode->lat;
                                update.lon = updatedNode->lon;
                                update.distance = updatedNode->distance;
                                update.azimuth = updatedNode->azimuth;
                                update.snr = updatedNode->snr;
                                update.lastSeenAge = millis() - updatedNode->lastSeen;
                                
                                bleManager.sendNodeUpdate(update);
                            }
 
                            // Вежливость: Если это новый узел, планируем ответное приветствие (NodeInfo),
                            // чтобы он узнал о нашем существовании
                            if (isNewNode && rxHeader.senderId != myNodeId) {
                                uint32_t currentMillis = millis();
                                uint32_t jitterMs = random(MIN_GREETING_NODEINFO_JITTER, MAX_GREETING_NODEINFO_JITTER); 
                                lastHeartbeatTime = currentMillis - HEARTBEAT_INTERVAL_MS + jitterMs;
                                LOG_INFO("SYS", "New Node %d discovered! NodeInfo reply scheduled", rxHeader.senderId);
                                
                                // Сохраняем обновленную базу в NVS
                                settingsManager.saveNodesSnapshot(nodeDB);
                            } 
 
                         // --- ПРИНЯТИЕ РЕШЕНИЯ О РЕТРАНСЛЯЦИИ ---
                         // Вызываем методы "Таможни" (Retranslation)
                         if (router.shouldRetransmit(rxHeader, nodeDB, myNodeType, currentSpeed)) {
                             uint8_t senderRole = NODE_STALKER; 
                             const NodeRecord* senderNode = nodeDB.getNode(rxHeader.senderId);
                             if (senderNode != nullptr) {
                                 senderRole = senderNode->type;
                             }
                             
                             // Вычисляем Adaptive Jitter и ставим в очередь TxManager
                             uint32_t calculatedJitter = calculateRelayJitter(myNodeType, senderRole, currentSNR);
                             txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader), payloadLen, calculatedJitter);
                         } 
                     } 
                 } 
             } 
         } 
         radio.startReceive(); // Возвращаем радио в режим приема
     } 
 
    // === АДАПТИВНАЯ ОТПРАВКА КООРДИНАТ (SMART TX) ===
    if (gps.isValid()) {
        bool shouldTransmit = false;
        const NodeRecord* myRecord = nodeDB.getNode(myNodeId);
        
        // Вычисляем смещение относительно нашей последней переданной позиции
        float distFromLastTx = (myRecord != nullptr) ? myRecord->distance : 0.0f;
        uint32_t now = millis();
 
        // Логика "В движении" (Дистанция > порога И Скорость > порога)
        if (distFromLastTx > MIN_MOVEMENT_METERS && currentSpeed > MIN_SPEED_KMPH) {
            shouldTransmit = true;
        // Логика "Крадущийся" (Безусловная отправка при сильном смещении без учета скорости)
        } else if (distFromLastTx > SNEAK_MOVEMENT_METERS) {
            shouldTransmit = true;
        } 
 
        // Проверка таймеров отправки
        if (shouldTransmit && (now - lastTxTime >= settingsManager.settings.txIntervalMoving)) {
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0); 
            LOG_INFO("ACTION", "Adaptive TX (Moving): Dist: %.1fm", distFromLastTx);
            lastTxTime = now;
        } else if (now - lastTxTime >= settingsManager.settings.txIntervalStill) {
            // Если стоим на месте, отправляем Heartbeat-координаты по длинному таймеру
            txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
            nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0);
            LOG_INFO("ACTION", "Adaptive TX (Still Heartbeat)");
            lastTxTime = now;
        } 
    } else {
        // Заглушка, если нет GPS
        if (millis() - lastTxTime >= settingsManager.settings.txIntervalStill) {
            LOG_WARN("TX", "Skip TX: GPS location not valid.");
            lastTxTime = millis(); 
        } 
    } 
 
     // Обработка очереди на передачу в радиоэфир (Освобождение CSMA/CA)
     txManager.processQueue();
 
     // === ОБНОВЛЕНИЕ ЭКРАНА И ФОНОВЫХ РАСЧЕТОВ (Раз в секунду) ===
     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         
         display.toggleLed(); // Мигаем светодиодом жизни
         
         int sats = gps.getSatellites();
 
         // Получение последней цели
         uint8_t currentTargetId = packetManager.getLastTargetId();
         const NodeRecord* targetNode = nodeDB.getNode(currentTargetId);
         
         // Проверка валидности захваченной цели
         bool isTargetValid = (targetNode != nullptr && targetNode->isActive);
 
         if (!isTargetValid && currentTargetId != 0) {
             packetManager.clearLastTargetId();
             currentTargetId = 0;
         } 
 
         // Распаковка упакованных координат
         if (gps.isValid()) {
             float currentLat = gps.getLat();
             // Динамический пересчет масштаба при смещении по широте (> 1 градуса)
             if (!isLonScaleSet || abs(currentLat - lastScaleLat) > 1.0f) {
                 packer.updateLonScale(currentLat); // Устанавливаем масштаб по широте
                 lastScaleLat = currentLat;
                 isLonScaleSet = true;
                 LOG_INFO("SYS", "Longitude scale updated for Lat: %.4f", currentLat);
             } 
 
             if (!isFastTracker) {
                 for (int i = 1; i < 255; i++) {
                     const NodeRecord* node = nodeDB.getNode(i);
                     if (node != nullptr && node->isActive) {
                         
                         // Если у нас есть сырые запакованные координаты, но нет распакованных float
                         if (node->packedCoords != 0 && node->lat == 0.0f && node->lon == 0.0f) {
                             float unpLat, unpLon;
                             packer.unpack(node->packedCoords, gps.getLat(), gps.getLon(), unpLat, unpLon);
                             nodeDB.updateNodeCoords(i, unpLat, unpLon, node->packedCoords, false);
                         } 
                         
                         // Пересчет дистанции и азимута
                         if (node->lat != 0.0f || node->lon != 0.0f) {
                             float d = gps.distanceTo(node->lat, node->lon);
                             float a = gps.courseTo(node->lat, node->lon);
                             nodeDB.updateNodeDistanceAzimuth(i, d, a);
                         } 
                     } 
                 } 
             } 
         } 
 
         // Формирование данных для строки цели на дисплее
         int targetDist = isTargetValid ? (int)targetNode->distance : 0;
         int targetAzimuth = isTargetValid ? (int)targetNode->azimuth : 0;
         int targetQuality = isTargetValid ? getConnectionQuality(targetNode->nodeId) : 0;
 
         BleStatus currentBleStatus = bleManager.getBleStatus();
 
         display.updateMainScreen(bleManager.macSuffix, gps.isValid(), sats, myNodeId, myMsgSeq, 
                                  nodeDB.getActiveNodesCount(), isTargetValid, currentTargetId, 
                                  targetDist, targetAzimuth, targetQuality, 
                                  currentBleStatus);
     } 
 } //MAIN.CPP