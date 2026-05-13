/**
 * Project: Naviga-Dongle
 * File: main.cpp
 * Version: 1.46
 * Изменение: Использование bleManager.sendNodeCoords в Smart TX и
 * bleManager.sendNodeInfo при смене Identity.
 */

#include "BleManager.h"
#include "CoordProcessor.h"
#include "DisplayManager.h"
#include "GeoPacker.h"
#include "GpsManager.h"
#include "NavigaProtocol.h"
#include "NetworkManager.h"
#include "NodeDatabase.h"
#include "PacketManager.h"
#include "PowerManager.h"
#include "RadioManager.h"
#include "Retranslation.h"
#include "SettingsManager.h"
#include "TxManager.h"
#include "configuration.h"
#include "logger.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

uint32_t networkScanDuration = 30000;
uint8_t myNodeId = 0;
uint8_t myMsgSeq = 0;
uint8_t myNodeType = NODE_RELAY;

PowerManager power;
DisplayManager display(0x3c, I2C_SDA, I2C_SCL);
GpsManager gps;
RadioManager radio;
GeoPacker packer;
NodeDatabase nodeDB;
Retranslation router;
BleManager bleManager;
PacketManager packetManager(nodeDB, gps, packer, bleManager);
TxManager txManager(radio, packer, myNodeId, myMsgSeq);
NetworkManager networkManager(radio, nodeDB, display, gps, txManager,
                              bleManager, router, packetManager);
CoordProcessor coordProcessor;

volatile bool receivedFlag = false;
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) { receivedFlag = true; }

uint32_t lastTxTime = 0;
uint32_t lastGpsLogTime = 0;
uint32_t lastCleanupTime = 0;
uint32_t lastHeartbeatTime = 0;
uint32_t lastTopologyUpdateTime = 0;
uint32_t lastTelemetryTime = 0;

void updateScreenCb(String l1, String l2, String l3, String l4) {
  display.showStatus(l1, l2, l3, l4);
}
void cycleGpsPowerCb() { power.cycleGpsPower(); }

void setup() {
  delay(500);
  Serial.begin(115200);
  LOG_INFO("SYS", "--- DONGLE BOOT START v1.46 ---");
#ifdef BOARD_T_BEAM_V11
  pinMode(LORA_ONBOARD_CS, OUTPUT);
  digitalWrite(LORA_ONBOARD_CS, HIGH);
#endif
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
  settingsManager.init();
  myNodeId = settingsManager.settings.nodeId;
  myNodeType = settingsManager.settings.nodeType;
  settingsManager.loadNodesSnapshot(nodeDB);
  nodeDB.ageAllNodes(settingsManager.settings.nodeConnectionTimeout + 1000);
  char myName[24];
  strncpy(myName, settingsManager.settings.nodeName, 23);
  myName[23] = '\0';
  nodeDB.addNode(myNodeId);
  nodeDB.updateNodeInfo(myNodeId, myName, myNodeType);
  if (myNodeType == NODE_RELAY)
    gps.setStaticLocation(RELAY_STATIC_LAT, RELAY_STATIC_LON);
  radio.init(setFlag);
  if (myNodeId == 0 || !settingsManager.settings.isConfigured) {
    networkManager.scanNetwork(false, networkScanDuration, myNodeId);
    settingsManager.settings.nodeId = myNodeId;
    settingsManager.settings.isConfigured = true;
    settingsManager.save();
  } else
    networkManager.scanNetwork(true, networkScanDuration, myNodeId);
  txManager.sendNodeInfo(myName, myNodeType, TX_NORMAL);
  lastTxTime = millis();
}

void loop() {
  uint32_t currentMillis = millis();
  float currentSpeed = gps.getSpeed();

  if (bleManager.requestIdentitySync) {
    bleManager.requestIdentitySync = false;
    bleManager.sendIdentity(settingsManager.settings.nodeId,
                            settingsManager.settings.nodeName,
                            settingsManager.settings.nodeType);
  }
  if (bleManager.requestSysConfigSync) {
    bleManager.requestSysConfigSync = false;
    bleManager.sendSysConfig(settingsManager.settings.txIntervalMoving,
                             settingsManager.settings.txIntervalStill,
                             settingsManager.settings.nodeConnectionTimeout,
                             settingsManager.settings.nodeActiveTimeoutMs);
  }
  if (bleManager.requestFullSync) {
    bleManager.requestFullSync = false;
    for (int i = 1; i < 255; i++) {
      const NodeRecord *node = nodeDB.getNode(i);
      if (node != nullptr && node->isActive) {
        BleEvtNodeUpdate update;
        update.opCode = EVT_NODE_UPDATE;
        update.nodeId = node->nodeId;
        update.nodeRole = node->type;
        strncpy(update.nodeName, node->nodeName, 23);
        update.nodeName[23] = '\0';
        update.lat = node->lat;
        update.lon = node->lon;
        update.snr = node->snr;
        update.lastSeenAge = millis() - node->lastSeen;
        bleManager.sendNodeUpdate(update);
        delay(5);
      }
    }
  }
  if (bleManager.hasNewIdentity) {
    bleManager.hasNewIdentity = false;
    myNodeId = bleManager.newIdentity.myNodeId;
    myNodeType = bleManager.newIdentity.myRole;
    settingsManager.settings.nodeId = myNodeId;
    settingsManager.settings.nodeType = myNodeType;
    strncpy(settingsManager.settings.nodeName, bleManager.newIdentity.myName,
            23);
    settingsManager.save();
    txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType,
                           TX_CRITICAL);
    nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName,
                          myNodeType);
    // ИЗМЕНЕНИЕ 1.46: Немедленный частичный апдейт инфо по BLE
    bleManager.sendNodeInfo(myNodeId, myNodeType,
                            settingsManager.settings.nodeName);
  }
  if (bleManager.hasNewSysConfig) {
    bleManager.hasNewSysConfig = false;
    settingsManager.settings.txIntervalMoving =
        bleManager.newSysConfig.txIntervalMoving;
    settingsManager.settings.txIntervalStill =
        bleManager.newSysConfig.txIntervalStill;
    settingsManager.settings.nodeConnectionTimeout =
        bleManager.newSysConfig.nodeConnectionTimeout;
    settingsManager.settings.nodeActiveTimeoutMs =
        bleManager.newSysConfig.nodeActiveTimeoutMs;
    settingsManager.save();
  }
  if (bleManager.requestClearDB) {
    bleManager.requestClearDB = false;
    for (int i = 1; i < 255; i++) {
      if (i != myNodeId)
        nodeDB.removeNode(i);
    }
    settingsManager.saveNodesSnapshot(nodeDB);
  }
  if (bleManager.requestReset) {
    settingsManager.factoryReset();
    delay(500);
    ESP.restart();
  }

  bool isFastTracker =
      (myNodeType == NODE_TRACKER && currentSpeed > TRACKER_FAST_SPEED_KMPH);
  if (gps.isValid() &&
      (currentMillis - lastTopologyUpdateTime > TOPOLOGY_UPDATE_INTERVAL_MS)) {
    if (!isFastTracker)
      nodeDB.updateTopology();
    lastTopologyUpdateTime = currentMillis;
  }
  if (currentMillis - lastCleanupTime > CLEANUP_INTERVAL_MS) {
    nodeDB.cleanup(myNodeId);
    lastCleanupTime = currentMillis;
  }
  if (currentMillis - lastHeartbeatTime > HEARTBEAT_INTERVAL_MS) {
    if (myNodeId != 0) {
      txManager.sendNodeInfo(settingsManager.settings.nodeName, myNodeType,
                             TX_NORMAL);
      nodeDB.updateNodeInfo(myNodeId, settingsManager.settings.nodeName,
                            myNodeType);
    }
    lastHeartbeatTime = currentMillis;
  }
  gps.update();
  if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = currentMillis;
    if (bleManager.getBleStatus() == BLE_CONNECTED)
      bleManager.sendMyStatus(gps.isValid(), gps.getSatellites(),
                              power.getBatteryPercent(),
                              power.getBatteryVoltage());
  }
  if (receivedFlag) {
    noInterrupts();
    receivedFlag = false;
    interrupts();
    size_t len = radio.getPacketLength();
    if (len >= sizeof(NavigaHeader)) {
      uint8_t rxBuffer[256];
      if (radio.readData(rxBuffer, len) == RADIOLIB_ERR_NONE) {
        float currentSNR = radio.getSNR();
        NavigaHeader rxHeader;
        memcpy(&rxHeader, rxBuffer, sizeof(NavigaHeader));
        if (rxHeader.relayId == myNodeId ||
            (rxHeader.senderId == myNodeId &&
             abs((int8_t)(myMsgSeq - rxHeader.msgSeq)) > 10))
          networkManager.handleCollision(myNodeId, myMsgSeq, myNodeType);
        if (rxHeader.relayId != myNodeId)
          nodeDB.updateNodeSNR(rxHeader.relayId, currentSNR);
        if (router.isValidPacket(rxHeader.getType(),
                                 len - sizeof(NavigaHeader))) {
          if (router.isDuplicate(rxHeader.senderId, rxHeader.msgSeq)) {
            if (!nodeDB.hasNodesInOppositeDirection(rxHeader.relayId))
              txManager.abortRelay(rxHeader.senderId, rxHeader.msgSeq);
          } else {
            bool isNew = !nodeDB.isNodeActive(rxHeader.senderId);
            packetManager.processPacket(rxHeader,
                                        rxBuffer + sizeof(NavigaHeader),
                                        len - sizeof(NavigaHeader));
            if (isNew && rxHeader.senderId != myNodeId) {
              lastHeartbeatTime =
                  millis() - HEARTBEAT_INTERVAL_MS + random(120000, 300000);
              settingsManager.saveNodesSnapshot(nodeDB);
            }
            if (router.shouldRetransmit(rxHeader, nodeDB, myNodeType,
                                        currentSpeed)) {
              uint8_t sR = NODE_STALKER;
              const NodeRecord *sn = nodeDB.getNode(rxHeader.senderId);
              if (sn)
                sR = sn->type;
              txManager.enqueueRelay(rxHeader, rxBuffer + sizeof(NavigaHeader),
                                     len - sizeof(NavigaHeader),
                                     networkManager.calculateRelayJitter(
                                         myNodeType, sR, currentSNR));
            }
          }
        }
      }
    }
    radio.startReceive();
  }
  if (gps.isValid()) {
    const NodeRecord *myR = nodeDB.getNode(myNodeId);
    float dLast = (myR != nullptr) ? myR->distance : 0.0f;
    bool shouldTx =
        (dLast > MIN_MOVEMENT_METERS && currentSpeed > MIN_SPEED_KMPH) ||
        (dLast > SNEAK_MOVEMENT_METERS);
    if ((shouldTx && (currentMillis - lastTxTime >=
                      settingsManager.settings.txIntervalMoving)) ||
        (currentMillis - lastTxTime >=
         settingsManager.settings.txIntervalStill)) {
      txManager.sendCoords(gps.getLat(), gps.getLon(), TX_HIGH);
      nodeDB.updateNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0);
      // ИЗМЕНЕНИЕ 1.46: Частичный апдейт своих координат по BLE
      if (bleManager.getBleStatus() == BLE_CONNECTED)
        bleManager.sendNodeCoords(myNodeId, gps.getLat(), gps.getLon(), 0.0f);
      lastTxTime = currentMillis;
    }
  }
  txManager.processQueue();
  if (currentMillis - lastGpsLogTime >= gpsUpdateInterval) {
    lastGpsLogTime = currentMillis;
    display.toggleLed();
    coordProcessor.process(nodeDB, gps, packer, isFastTracker);
    uint8_t tId = packetManager.getLastTargetId();
    const NodeRecord *tN = nodeDB.getNode(tId);
    bool tV = (tN != nullptr && tN->isActive);
    if (!tV && tId != 0) {
      packetManager.clearLastTargetId();
      tId = 0;
    }
    display.updateMainScreen(
        bleManager.macSuffix, gps.isValid(), gps.getSatellites(), myNodeId,
        myMsgSeq, nodeDB.getActiveNodesCount(), tV, tId,
        tV ? (int)tN->distance : 0, tV ? (int)tN->azimuth : 0,
        tV ? networkManager.getConnectionQuality(tId, myNodeId) : 0,
        bleManager.getBleStatus());
  }
} // main.cpp
