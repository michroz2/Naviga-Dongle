/**
 * File: RadioManager.cpp
 * Version: 1.22 Изменение: Единая конфигурация SX1268 (E22) для плат T-Beam и T-Energy S3.
 * Description: Реализация класса управления LoRa.
 */
 #include "RadioManager.h"

 // Конструктор: динамически выделяем память под Module и передаем его в объект радио.
 // Используется универсальный LORA_IRQ, который в config.h настроен на нужный пин для каждой платы.
 RadioManager::RadioManager() : 
     _mod(new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY)), 
     _radio(_mod) {
 }
 
 bool RadioManager::init(void (*isr)(void)) {
     int state = _radio.begin(433.0);
     if (state == RADIOLIB_ERR_NONE) {
 
 #ifdef LORA_CHIP_SX1268
         // Единые настройки SX1268 (E22) для обеих аппаратных платформ
         _radio.setDio2AsRfSwitch(false);    
         _radio.setBandwidth(125.0);         
         _radio.setSpreadingFactor(9);       
         _radio.setCodingRate(5);            
         _radio.setSyncWord(0x2B);           
         _radio.setPreambleLength(16);       
         _radio.setOutputPower(22);          
         _radio.setCurrentLimit(140);        
         _radio.setTCXO(1.8);
         
         // Эта строка теперь работает для обеих плат, так как T-Energy S3 тоже имеет эти пины
         _radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         
         _radio.setRxBoostedGainMode(true);
 #endif
 
         // Подключаем аппаратное прерывание и стартуем
         _radio.setPacketReceivedAction(isr); 
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