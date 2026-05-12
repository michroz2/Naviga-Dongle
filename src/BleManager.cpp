/**
 * File: BleManager.cpp
 * Version: 1.43
 * Изменение: Обновление версии файла (реализация симметричной выгрузки координат донгла).
 * Description: Реализация менеджера Bluetooth.
 */

 #include "BleManager.h"
 #include "logger.h" 
 #include <esp_mac.h> 

 // Обработчики событий подключения/отключения смартфона
 class BleManager::ServerCallbacks : public NimBLEServerCallbacks {
     BleManager* _manager;
 public:
     ServerCallbacks(BleManager* manager) : _manager(manager) {}
     
     void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
         _manager->_isConnected = true;
         LOG_INFO("BLE", "Smartphone connected!");
         // При подключении NimBLE автоматически останавливает Advertising.
         // Это делает Донгл эксклюзивным и невидимым для других сканеров.
     }

     void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
         _manager->_isConnected = false;
         LOG_INFO("BLE", "Smartphone disconnected. Restarting advertising...");
         // При отключении перезапускаем Advertising, чтобы снова стать видимыми
         NimBLEDevice::startAdvertising(); 
     }
 };

 // Обработчики получения данных со смартфона (Свойство WRITE)
 class BleManager::RxCallbacks : public NimBLECharacteristicCallbacks {
     BleManager* _manager;
 public:
     RxCallbacks(BleManager* manager) : _manager(manager) {}
 
     void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
         // Получаем бинарные данные от приложения Оператора
         std::string rxValue = pCharacteristic->getValue();
         if (rxValue.length() > 0) {
             uint8_t opCode = rxValue[0]; // Читаем первый байт (Код операции)
 
             // В зависимости от кода операции, копируем данные в буферы и поднимаем флаги
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
     requestIdentitySync(false), requestSysConfigSync(false) {
         macSuffix[0] = '\0'; // Инициализируем пустой строкой
     }
 
 void BleManager::init() {
     // Считываем аппаратный MAC Bluetooth (Bluetooth-интерфейса чипа ESP)
     // Берем 2 последних байта (4 шестнадцатеричных символа) для уникальности
     uint8_t mac[6];
     esp_read_mac(mac, ESP_MAC_BT);
     snprintf(macSuffix, sizeof(macSuffix), "%02X%02X", mac[4], mac[5]);
     
     // Формируем динамическое имя для рекламных пакетов (Например: Naviga-7A3E)
     char devName[20];
     snprintf(devName, sizeof(devName), "Naviga-%s", macSuffix);

     NimBLEDevice::init(devName);
     
     pServer = NimBLEDevice::createServer();
     pServer->setCallbacks(new ServerCallbacks(this));
 
     // Создаем сервис по UUID
     NimBLEService* pService = pServer->createService(SERVICE_UUID);
 
     // Создаем характеристику TX для уведомлений телефона (NOTIFY)
     pTxCharacteristic = pService->createCharacteristic(
         CHARACTERISTIC_UUID_TX,
         NIMBLE_PROPERTY::NOTIFY
     );
 
     // Создаем характеристику RX для записи со смартфона (WRITE)
     pRxCharacteristic = pService->createCharacteristic(
         CHARACTERISTIC_UUID_RX,
         NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
     );
     pRxCharacteristic->setCallbacks(new RxCallbacks(this));
     
     pServer->start(); 
 
     // Настраиваем Advertising (видимость в радиоэфире)
     NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
     pAdvertising->setName(devName); // Явно вещаем динамическое имя
     pAdvertising->addServiceUUID(SERVICE_UUID);
     
     pAdvertising->start();
 
     LOG_INFO("SYS", "BLE Initialized. Name: %s", devName);
 }
      
 BleStatus BleManager::getBleStatus() {
     if (_isConnected) return BLE_CONNECTED;
     return BLE_UNPAIRED; 
 }
 
 void BleManager::process() {
     // Вся логика выполнения команд теперь безопасно выполняется в loop() в main.cpp,
     // чтобы избежать проблем с многозадачностью и прерываниями внутри коллбэков.
 }
 
 // Упаковка и отправка структуры BleIdentity через характеристику TX
 void BleManager::sendIdentity(uint8_t nodeId, const char* name, uint8_t role) {
     if (!_isConnected) return;
     
     BleIdentity packet;
     packet.opCode = EVT_IDENTITY;
     packet.myNodeId = nodeId;
     packet.myRole = role;
     strncpy(packet.myName, name, sizeof(packet.myName) - 1);
     packet.myName[sizeof(packet.myName) - 1] = '\0'; 
 
     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleIdentity));
     pTxCharacteristic->notify(); // Отправка уведомления на смартфон
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
 
 // ИЗМЕНЕНИЕ 1.40: Структура пакета в BleProtocol.h изменилась (удалены distance/azimuth).
 // Здесь мы просто передаем подготовленную структуру в характеристику.
 void BleManager::sendNodeUpdate(const BleEvtNodeUpdate& nodeData) {
     if (!_isConnected) return;
     pTxCharacteristic->setValue((uint8_t*)&nodeData, sizeof(BleEvtNodeUpdate));
     pTxCharacteristic->notify();
 } 

 // ИЗМЕНЕНИЕ 1.41: Реализация отправки уведомления об удалении узла
 void BleManager::sendNodeDelete(uint8_t nodeId) {
     if (!_isConnected) return;

     BleEvtNodeDelete packet;
     packet.opCode = EVT_NODE_DELETE;
     packet.nodeId = nodeId;

     pTxCharacteristic->setValue((uint8_t*)&packet, sizeof(BleEvtNodeDelete));
     pTxCharacteristic->notify();
     LOG_INFO("BLE", "Sent NODE_DELETE for ID %d to App", nodeId);
 }

 // ИЗМЕНЕНИЕ 1.34: Упаковка и отправка телеметрии о состоянии Донгла
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