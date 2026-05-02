/**
 * File: BleProtocol.h
 * Version: 1.29
 * Description: Описание структур данных для BLE GATT с разделением Identity и Config (UC-04 Pairing).
 */

 #ifndef BLE_PROTOCOL_H
 #define BLE_PROTOCOL_H
 
 #include <stdint.h>
 
 #pragma pack(push, 1)
 
 // ==========================================================
 // 1. КОДЫ ОПЕРАЦИЙ (OpCodes)
 // ==========================================================
 enum BleOpCode : uint8_t {
     // Оператор -> Донгл (Запись/Команды)
     CMD_SET_IDENTITY    = 0x01, // Задать новые Имя и Роль
     CMD_SET_SYS_CONFIG  = 0x02, // Задать системные тайминги и настройки
     CMD_ACTION_RESET    = 0x03, // Hard Reset
     CMD_ACTION_CLEAR_DB = 0x04, // Очистить базу узлов
     
     // Запросы от Оператора к Донглу (при подключении)
     CMD_REQ_FULL_SYNC   = 0x05, // Запросить базу соседей
     CMD_REQ_IDENTITY    = 0x06, // Запросить текущие Имя и Роль Донгла
     CMD_REQ_SYS_CONFIG  = 0x07, // Запросить текущие настройки Донгла
 
     // Донгл -> Оператор (Уведомления)
     EVT_MY_STATUS       = 0x10, // Телеметрия (GPS, Батарея)
     EVT_NODE_UPDATE     = 0x11, // Обновление по соседу
     EVT_IDENTITY        = 0x12, // Ответ: Мои текущие Имя и Роль
     EVT_SYS_CONFIG      = 0x13  // Ответ: Мои текущие настройки
 };
 
 // ==========================================================
 // 2. ИДЕНТИФИКАТОР УЗЛА (Identity)
 // ==========================================================
 
 // Пакет от Оператора (CMD_SET_IDENTITY) или ответ от Донгла (EVT_IDENTITY)
 struct BleIdentity {
     uint8_t opCode;             // CMD_SET_IDENTITY или EVT_IDENTITY
     uint8_t myNodeId;           // ID узла (изменяет Донгл при коллизиях)
     char myName[12];            // Имя узла
     uint8_t myRole;             // 0-Relay, 1-Stalker, 2-Tracker
 };
 
 // ==========================================================
 // 3. СИСТЕМНЫЕ НАСТРОЙКИ (SysConfig)
 // ==========================================================
 
 // Пакет от Оператора (CMD_SET_SYS_CONFIG) или ответ от Донгла (EVT_SYS_CONFIG)
 struct BleSysConfig {
     uint8_t opCode;             // CMD_SET_SYS_CONFIG или EVT_SYS_CONFIG
     uint32_t txIntervalMoving;  // Интервал TX в движении (мс)
     uint32_t txIntervalStill;   // Интервал TX на стоянке (мс)
     
     // ИЗМЕНЕНИЕ 1.29: Добавлены таймауты
     uint32_t nodeConnectionTimeout; // Таймаут потери связи (мс)
     uint32_t nodeActiveTimeoutMs;   // Таймаут очистки из базы (мс)
 };
 
 // ==========================================================
 // 4. ТЕЛЕМЕТРИЯ И ТОПОЛОГИЯ (Без изменений)
 // ==========================================================
 
 struct BleEvtMyStatus {
     uint8_t opCode;             // EVT_MY_STATUS (0x10)
     uint8_t gpsValid;           
     uint8_t satellites;         
     uint8_t batteryPercent;     
     uint8_t txQueueSize;        
 };
 
 struct BleEvtNodeUpdate {
     uint8_t opCode;             // EVT_NODE_UPDATE (0x11)
     uint8_t nodeId;             
     uint8_t nodeRole;           
     char nodeName[12];          
     float lat;                  
     float lon;                  
     float distance;             
     float azimuth;              
     float snr;                  
     uint32_t lastSeenAge;       
 };
 
 #pragma pack(pop)
 
 #endif // BLE_PROTOCOL_H