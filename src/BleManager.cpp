/**
 * File: BleManager.cpp
 * Version: 1.29 
 * Description: Реализация BLE-менеджера. Интеграция UC-04 (Pairing Sync).
 */

 #include "BleManager.h"
 #include "logger.h" 
 
 // --- Внутренние классы Callbacks ---
 
 class BleManager::ServerCallbacks : public NimBLEServerCallbacks {
     BleManager* _manager;
 public:
     ServerCallbacks(BleManager* manager) : _manager(manager) {}
     
     void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
         _manager->_isConnected = true;
         LOG_INFO("BLE", "Smartphone connected!");
         // NimBLE автоматически останавливает рекламу при подключении
     }
 
     void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
         _manager->_isConnected = false;
         LOG_INFO("BLE", "Smartphone disconnected. Restarting advertising...");
         NimBLEDevice::startAdvertising(); // Снова становимся видимыми
     }
 };
 
 class BleManager::RxCallbacks : public NimBLECharacteristicCallbacks {
     BleManager* _manager;
 public:
     RxCallbacks(BleManager* manager) : _manager(manager) {}
 
     void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
         std::string rxValue = pCharacteristic->getValue();
         if (rxValue.length() > 0) {
             uint8_t opCode = rxValue[0]; // Читаем первый байт (Код операции)
 
             switch (opCode) {
                 case CMD_SET_IDENTITY:
                     if (rxValue.length() == sizeof(BleIdentity)) {
                         memcpy(&_manager->newIdentity, rxValue.data(), sizeof(BleIdentity));
                         _manager->hasNewIdentity = true; 
                         LOG_INFO("BLE_RX", "Received new Identity settings");
                     }
                     break;
 
                 case CMD_SET_SYS_CONFIG:
                     if (rxValue.length() == sizeof(BleSysConfig)) {
                         memcpy(&_manager->newSysConfig, rxValue.data(), sizeof(BleSysConfig));
                         _manager->hasNewSysConfig = true; 
                         LOG_INFO("BLE_RX", "Received new System Config");
                     }
                     break;
 
                 case CMD_REQ_FULL_SYNC:
                     _manager->requestFullSync = true;
                     LOG_INFO("BLE_RX", "Smartphone requested FULL SYNC (Topology)");
                     break;
                     
                 // ИЗМЕНЕНИЕ 1.29: Обработка запросов при первом сопряжении
                 case CMD_REQ_IDENTITY:
                     _manager->requestIdentitySync = true;
                     LOG_INFO("BLE_RX", "App requested Identity Sync (UC-04)");
                     break;
                     
                 case CMD_REQ_SYS_CONFIG:
                     _manager->requestSysConfigSync = true;
                     LOG_INFO("BLE_RX", "App requested SysConfig Sync (UC-04)");
                     break;
 
                 case CMD_ACTION_RESET:
                     _manager->requestReset = true;
                     LOG_INFO("BLE_RX", "Action: Hardware Reset Requested");
                     break;
 
                 case CMD_ACTION_CLEAR_DB:
                     _manager->requestClearDB = true;
                     LOG_INFO("BLE_RX", "Action: Clear Database Requested");
                     break;
                 
                 default:
                     LOG_WARN("BLE_RX", "Unknown OpCode: 0x%02X", opCode);
                     break;
             }
         }
     }
 };
 
 // --- Основной класс BleManager ---
 
 BleManager::BleManager() : 
     pServer(nullptr), pTxCharacteristic(nullptr), pRxCharacteristic(nullptr),
     _isConnected(false), hasNewIdentity(false), hasNewSysConfig(false),
     requestFullSync(false), requestReset(false), requestClearDB(false),
     requestIdentitySync(false), requestSysConfigSync(false) {} // ИЗМЕНЕНИЕ: Инициализация флагов
 
 void BleManager::init() {
     NimBLEDevice::init(BLE_DEVICE_NAME);
     
     // Оптимизация мощности передатчика BLE для пробиваемости через корпус
     NimBLEDevice::setPower(ESP_PWR_LVL_P9); 
 
     pServer = NimBLEDevice::createServer();
     pServer->setCallbacks(new ServerCallbacks(this));
 
     NimBLEService* pService = pServer->createService(SERVICE_UUID);
 
     // TX Характеристика (Донгл -> Смартфон) - Только NOTIFY
     pTxCharacteristic = pService->createCharacteristic(
         CHARACTERISTIC_UUID_TX,
         NIMBLE_PROPERTY::NOTIFY
     );
 
     // RX Характеристика (Смартфон -> Донгл) - Только WRITE
     pRxCharacteristic = pService->createCharacteristic(
         CHARACTERISTIC_UUID_RX,
         NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
     );
     pRxCharacteristic->setCallbacks(new RxCallbacks(this));
     
     // Настройка рекламы (Advertising)
     NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
     pAdvertising->addServiceUUID(SERVICE_UUID);
     pAdvertising->start();
 
     LOG_INFO("SYS", "BLE Initialized. Advertising started.");
 }
 
 BleStatus BleManager::getBleStatus() {
     if (_isConnected) return BLE_CONNECTED;
     return BLE_UNPAIRED; 
 }
 
 void BleManager::process() {
     // Вся логика теперь безопасно выполняется в main.cpp
 }
 
 void BleManager::sendIdentity(uint8_t nodeId, const char* name, uint8_t role) {
     if (!_isConnected) return;
     
     BleIdentity packet;
     packet.opCode = EVT_IDENTITY;
     packet.myNodeId = nodeId;
     packet.myRole = role;
     strncpy(packet.myName, name, sizeof(packet.myName) - 1);
     packet.myName[sizeof(packet.myName) - 1] = '\0'; 
 
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleIdentity));
     pTxCharacteristic->notify();
 }
 
 // ИЗМЕНЕНИЕ 1.29: Обновленная сигнатура
 void BleManager::sendSysConfig(uint32_t txMoving, uint32_t txStill, uint32_t connTimeout, uint32_t activeTimeout) {
     if (!_isConnected) return;
 
     BleSysConfig packet;
     packet.opCode = EVT_SYS_CONFIG;
     packet.txIntervalMoving = txMoving;
     packet.txIntervalStill = txStill;
     packet.nodeConnectionTimeout = connTimeout;
     packet.nodeActiveTimeoutMs = activeTimeout;
 
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleSysConfig));
     pTxCharacteristic->notify();
 }
 
 void BleManager::sendNodeUpdate(const BleEvtNodeUpdate& nodeData) {
     if (!_isConnected) return;
     pTxCharacteristic->setValue((uint8_t*)&nodeData, sizeof(BleEvtNodeUpdate));
     pTxCharacteristic->notify();
 }