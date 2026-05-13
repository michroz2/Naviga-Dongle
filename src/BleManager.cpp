/**
 * File: BleManager.cpp
 * Version: 1.46
 * Изменение: Реализованы методы sendNodeCoords и sendNodeInfo для частичных обновлений.
 */

 #include "BleManager.h"
 #include "logger.h" 
 #include <esp_mac.h> 

 class BleManager::ServerCallbacks : public NimBLEServerCallbacks {
     BleManager* _manager;
 public:
     ServerCallbacks(BleManager* manager) : _manager(manager) {}
     
     void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
         _manager->_isConnected = true;
         LOG_INFO("BLE", "Smartphone connected!");
     }

     void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
         _manager->_isConnected = false;
         LOG_INFO("BLE", "Smartphone disconnected. Restarting advertising...");
         NimBLEDevice::startAdvertising(); 
     }
 };

 class BleManager::RxCallbacks : public NimBLECharacteristicCallbacks {
     BleManager* _manager;
 public:
     RxCallbacks(BleManager* manager) : _manager(manager) {}
 
     void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
         std::string rxValue = pCharacteristic->getValue();
         if (rxValue.length() > 0) {
             uint8_t opCode = rxValue[0];
             switch (opCode) {
                 case CMD_SET_IDENTITY:
                     if (rxValue.length() == sizeof(BleIdentity)) {
                         memcpy(&_manager->newIdentity, rxValue.data(), sizeof(BleIdentity));
                         _manager->hasNewIdentity = true; 
                     }
                     break;
                 case CMD_SET_SYS_CONFIG:
                     if (rxValue.length() == sizeof(BleSysConfig)) {
                         memcpy(&_manager->newSysConfig, rxValue.data(), sizeof(BleSysConfig));
                         _manager->hasNewSysConfig = true; 
                     }
                     break;
                 case CMD_REQ_FULL_SYNC: _manager->requestFullSync = true; break;
                 case CMD_REQ_IDENTITY:  _manager->requestIdentitySync = true; break;
                 case CMD_REQ_SYS_CONFIG: _manager->requestSysConfigSync = true; break;
                 case CMD_ACTION_RESET:   _manager->requestReset = true; break;
                 case CMD_ACTION_CLEAR_DB: _manager->requestClearDB = true; break;
             }
         }
     }
 };
 
 BleManager::BleManager() : 
     pServer(nullptr), pTxCharacteristic(nullptr), pRxCharacteristic(nullptr),
     _isConnected(false), hasNewIdentity(false), hasNewSysConfig(false),
     requestFullSync(false), requestReset(false), requestClearDB(false),
     requestIdentitySync(false), requestSysConfigSync(false) {
         macSuffix[0] = '\0';
     }
 
 void BleManager::init() {
     uint8_t mac[6];
     esp_read_mac(mac, ESP_MAC_BT);
     snprintf(macSuffix, sizeof(macSuffix), "%02X%02X", mac[4], mac[5]);
     char devName[20];
     snprintf(devName, sizeof(devName), "Naviga-%s", macSuffix);
     NimBLEDevice::init(devName);
     pServer = NimBLEDevice::createServer();
     pServer->setCallbacks(new ServerCallbacks(this));
     NimBLEService* pService = pServer->createService(SERVICE_UUID);
     pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
     pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
     pRxCharacteristic->setCallbacks(new RxCallbacks(this));
     pServer->start(); 
     NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
     pAdvertising->setName(devName);
     pAdvertising->addServiceUUID(SERVICE_UUID);
     pAdvertising->start();
     LOG_INFO("SYS", "BLE Initialized. Name: %s", devName);
 }
      
 BleStatus BleManager::getBleStatus() {
     return _isConnected ? BLE_CONNECTED : BLE_UNPAIRED;
 }
 
 void BleManager::process() {}
 
 void BleManager::sendIdentity(uint8_t nodeId, const char* name, uint8_t role) {
     if (!_isConnected) return;
     BleIdentity packet;
     packet.opCode = EVT_IDENTITY;
     packet.myNodeId = nodeId;
     packet.myRole = role;
     strncpy(packet.myName, name, 23);
     packet.myName[23] = '\0'; 
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleIdentity));
     pTxCharacteristic->notify();
 }
 
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

 // ИЗМЕНЕНИЕ 1.46: Частичное обновление координат (0x15)
 void BleManager::sendNodeCoords(uint8_t id, float lat, float lon, float snr) {
     if (!_isConnected) return;
     BleEvtNodeCoords packet;
     packet.opCode = EVT_NODE_COORDS;
     packet.nodeId = id;
     packet.lat = lat;
     packet.lon = lon;
     packet.snr = snr;
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtNodeCoords));
     pTxCharacteristic->notify();
 }

 // ИЗМЕНЕНИЕ 1.46: Частичное обновление информации об узле (0x16)
 void BleManager::sendNodeInfo(uint8_t id, uint8_t role, const char* name) {
     if (!_isConnected) return;
     BleEvtNodeInfo packet;
     packet.opCode = EVT_NODE_INFO;
     packet.nodeId = id;
     packet.nodeRole = role;
     strncpy(packet.nodeName, name, 23);
     packet.nodeName[23] = '\0';
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtNodeInfo));
     pTxCharacteristic->notify();
 }

 void BleManager::sendNodeDelete(uint8_t nodeId) {
     if (!_isConnected) return;
     BleEvtNodeDelete packet;
     packet.opCode = EVT_NODE_DELETE;
     packet.nodeId = nodeId;
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtNodeDelete));
     pTxCharacteristic->notify();
 }

 void BleManager::sendMyStatus(uint8_t gpsValid, uint8_t satellites, uint8_t batteryPercent, uint16_t batteryVoltage) {
     if (!_isConnected) return;
     BleEvtMyStatus packet;
     packet.opCode = EVT_MY_STATUS;
     packet.gpsValid = gpsValid;
     packet.satellites = satellites;
     packet.batteryPercent = batteryPercent;
     packet.batteryVoltage = batteryVoltage;
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtMyStatus));
     pTxCharacteristic->notify();
 } //BleManager.cpp