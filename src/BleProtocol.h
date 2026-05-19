/**
 * Project: Naviga-Dongle
 * File: BleProtocol.h
 * Version: 1.47.1
 * Description: Описание структур данных для BLE GATT. 
 * Изменение: Поле gpsValid в BleEvtMyStatus заменено на gpsState с поддержкой трех состояний.
 */

 #ifndef BLE_PROTOCOL_H
 #define BLE_PROTOCOL_H
 
 #include <stdint.h>
 
 #pragma pack(push, 1) // Отключение выравнивания памяти
 
 // ==========================================================
 // 1. КОДЫ ОПЕРАЦИЙ (OpCodes)
 // ==========================================================
 enum BleOpCode : uint8_t {
      CMD_SET_IDENTITY    = 0x01,
      CMD_SET_SYS_CONFIG  = 0x02,
      CMD_ACTION_RESET    = 0x03,
      CMD_ACTION_CLEAR_DB = 0x04,
      CMD_REQ_FULL_SYNC   = 0x05,
      CMD_REQ_IDENTITY    = 0x06,
      CMD_REQ_SYS_CONFIG  = 0x07,
      CMD_SET_ANCHOR_COORDS = 0x08,
 
      EVT_MY_STATUS       = 0x10,
      EVT_NODE_UPDATE     = 0x11, 
      EVT_IDENTITY        = 0x12,
      EVT_SYS_CONFIG      = 0x13,
      EVT_NODE_DELETE     = 0x14,
      EVT_NODE_COORDS     = 0x15, 
      EVT_NODE_INFO       = 0x16  
 };
 
 // НОВОЕ 1.47.1: Возможные состояния аппаратного GPS модуля для Телеметрии
 enum BleGpsState : uint8_t {
     GPS_STATE_SEARCHING = 0, // Железо есть, фикса нет (Ищем спутники)
     GPS_STATE_FIX_OK    = 1, // Железо есть, фикс есть (Всё отлично)
     GPS_STATE_NO_HW     = 2  // Железа нет (Слепое Реле)
 };
 
 // ==========================================================
 // 2. СТРУКТУРЫ ДАННЫХ
 // ==========================================================
 
 struct BleIdentity {
      uint8_t opCode;
      uint8_t myNodeId;
      char myName[24];
      uint8_t myRole;
 };
 
 struct BleSysConfig {
      uint8_t opCode;
      uint32_t txIntervalMoving;
      uint32_t txIntervalStill;
      uint32_t nodeConnectionTimeout;
      uint32_t nodeActiveTimeoutMs;
 };
 
 struct BleEvtMyStatus {
      uint8_t opCode;
      uint8_t gpsState; // ИЗМЕНЕНИЕ 1.47.1: Было gpsValid. Теперь принимает значения из BleGpsState.
      uint8_t satellites;
      uint8_t batteryPercent;
      uint16_t batteryVoltage;
 };
 
 struct BleEvtNodeUpdate {
      uint8_t opCode;
      uint8_t nodeId;
      uint8_t nodeRole;
      char nodeName[24];
      float lat;
      float lon;
      float snr;
      uint32_t lastSeenAge;
 };
 
 struct BleEvtNodeDelete {
      uint8_t opCode;
      uint8_t nodeId;
 };
 
 struct BleEvtNodeCoords {
      uint8_t opCode;  // 0x15
      uint8_t nodeId;
      float lat;
      float lon;
      float snr;
 };
 
 struct BleEvtNodeInfo {
      uint8_t opCode;  // 0x16
      uint8_t nodeId;
      uint8_t nodeRole;
      char nodeName[24];
 };
 
 struct BleCmdAnchorCoords {
      uint8_t opCode; // 0x08
      float lat;
      float lon;
 };
 
 #pragma pack(pop)
 
 #endif // BLE_PROTOCOL_H