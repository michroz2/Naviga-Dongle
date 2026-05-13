/**
 * File: BleProtocol.h
 * Version: 1.46.3
 * Изменение: Единый заголовочный файл протокола со статусами и структурами.
 * Description: Определение протокола связи.
 */

 #ifndef BLE_PROTOCOL_H
 #define BLE_PROTOCOL_H
 
 #include <Arduino.h>
 
 // Состояния BLE (единое определение)
 enum BleStatus {
     BLE_OFF,
     BLE_UNPAIRED,
     BLE_CONNECTED
 };
 
 #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
 #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
 #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
 
 enum BleOpCode {
     CMD_SET_IDENTITY    = 0x01,
     CMD_SET_SYS_CONFIG  = 0x02,
     CMD_ACTION_RESET    = 0x03,
     CMD_ACTION_CLEAR_DB = 0x04,
     CMD_REQ_FULL_SYNC   = 0x05,
     CMD_REQ_IDENTITY    = 0x06,
     CMD_REQ_SYS_CONFIG  = 0x07,
 
     EVT_MY_STATUS       = 0x10,
     EVT_NODE_UPDATE     = 0x11, 
     EVT_IDENTITY        = 0x12,
     EVT_SYS_CONFIG      = 0x13,
     EVT_NODE_DELETE     = 0x14,
     EVT_NODE_COORDS     = 0x15, 
     EVT_NODE_INFO       = 0x16  
 };
 
 #pragma pack(push, 1)
 
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
     uint8_t opCode;
     uint8_t nodeId;
     float lat;
     float lon;
     float snr;
 };
 
 struct BleEvtNodeInfo {
     uint8_t opCode;
     uint8_t nodeId;
     uint8_t nodeRole;
     char nodeName[24];
 };
 
 #pragma pack(pop)
 
 #endif // BleProtocol.h