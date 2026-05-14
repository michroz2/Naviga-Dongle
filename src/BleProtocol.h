/**
 * Project: Naviga-Dongle
 * File: BleProtocol.h
 * Version: 1.46.5
 * Description: Описание структур данных для BLE GATT. 
 * Изменение: Добавлены структуры дельта-обновлений (0x15, 0x16) согласно Контракту 1.46.4.
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
 
     EVT_MY_STATUS       = 0x10,
     EVT_NODE_UPDATE     = 0x11, // Полный пакет (для Full Sync)
     EVT_IDENTITY        = 0x12,
     EVT_SYS_CONFIG      = 0x13,
     EVT_NODE_DELETE     = 0x14,
     EVT_NODE_COORDS     = 0x15, // НОВОЕ: Дельта-координаты
     EVT_NODE_INFO       = 0x16  // НОВОЕ: Дельта-инфо (Имя/Роль)
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
     uint8_t gpsValid;
     uint8_t satellites;
     uint8_t batteryPercent;
     uint16_t batteryVoltage;
 };
 
 // Полный пакет обновления (43 байта) - используется только для Full Sync
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
 
 // НОВОЕ 1.46.5: Дельта-координаты (14 байт)
 struct BleEvtNodeCoords {
     uint8_t opCode;  // 0x15
     uint8_t nodeId;
     float lat;
     float lon;
     float snr;
 };
 
 // НОВОЕ 1.46.5: Дельта-инфо (27 байт)
 struct BleEvtNodeInfo {
     uint8_t opCode;  // 0x16
     uint8_t nodeId;
     uint8_t nodeRole;
     char nodeName[24];
 };
 
 #pragma pack(pop)
 
 #endif // BLE_PROTOCOL_H