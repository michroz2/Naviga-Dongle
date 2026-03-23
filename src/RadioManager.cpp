/**
 * File: RadioManager.cpp
 * Version: 1.0.0
 * Description: Реализация класса управления LoRa.
 */
 #include "RadioManager.h"

 // Конструктор: динамически выделяем память под Module и передаем его в SX1268
 RadioManager::RadioManager() : 
     _mod(new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY)), 
     _radio(_mod) {
 }
 
 bool RadioManager::init(void (*isr)(void)) {
     int state = _radio.begin(433.0);
     if (state == RADIOLIB_ERR_NONE) {
         _radio.setDio2AsRfSwitch(false);    
         _radio.setBandwidth(125.0);         
         _radio.setSpreadingFactor(9);       
         _radio.setCodingRate(5);            
         _radio.setSyncWord(0x2B);           
         _radio.setPreambleLength(16);       
         _radio.setOutputPower(22);          
         _radio.setCurrentLimit(140);        
         _radio.setTCXO(1.8);
         _radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         _radio.setRxBoostedGainMode(true);
         _radio.setPacketReceivedAction(isr); // Подключаем аппаратное прерывание
         _radio.startReceive();
         return true;
     }
     return false;
 }
 
 void RadioManager::startReceive() { _radio.startReceive(); }
 void RadioManager::standby() { _radio.standby(); }
 size_t RadioManager::getPacketLength() { return _radio.getPacketLength(); }
 int RadioManager::readData(uint8_t* buffer, size_t len) { return _radio.readData(buffer, len); }
 int RadioManager::transmit(uint8_t* buffer, size_t len) { return _radio.transmit(buffer, len); }
 float RadioManager::getSNR() { return _radio.getSNR(); }