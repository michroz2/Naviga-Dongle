/**
 * File: RadioManager.h
 * Version: 1.0.0
 * Description: Изолированный класс для управления радиомодулем LoRa (SX1268).
 */
 #ifndef RADIO_MANAGER_H
 #define RADIO_MANAGER_H
 
 #include <Arduino.h>
 #include <RadioLib.h>
 #include "configuration.h"
 
 class RadioManager {
 public:
     RadioManager();
 
     // Инициализация. Принимает функцию прерывания (коллбэк). Возвращает true при успехе.
     bool init(void (*isr)(void));
 
     // Проброс базовых методов RadioLib наружу
     void startReceive();
     void standby();
     size_t getPacketLength();
     int readData(uint8_t* buffer, size_t len);
     int transmit(uint8_t* buffer, size_t len);
     float getSNR();
 
 private:
     Module* _mod;
     SX1268 _radio;
 };
 
 #endif // RADIO_MANAGER_H