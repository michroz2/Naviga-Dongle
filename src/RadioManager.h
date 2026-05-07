/**
 * File: RadioManager.h
 * Version: 1.31 Изменение: Подготовка к ручному управлению антенным коммутатором.
 * Description: Изолированный класс (обертка) для управления радиомодулем LoRa.
 */
 #ifndef RADIO_MANAGER_H
 #define RADIO_MANAGER_H
 
 #include <Arduino.h>
 #include <RadioLib.h>
 #include "configuration.h"
 
 class RadioManager {
 public:
     RadioManager();
 
     // Инициализация. Принимает функцию прерывания (коллбэк ISR). Возвращает true при успехе.
     bool init(void (*isr)(void));
 
     // Проброс базовых методов RadioLib наружу для использования в main.cpp
     void startReceive();
     void standby();
     size_t getPacketLength();
     int readData(uint8_t* buffer, size_t len);
     int transmit(uint8_t* buffer, size_t len);
     float getSNR();
 
 private:
     Module* _mod;
 
     // Инициализация класса из RadioLib на основе конфигурации платы
 #ifdef LORA_CHIP_SX1268
     SX1268 _radio;
 #elif defined(LORA_CHIP_SX1278)
     SX1278 _radio; // Оставлено для будущей совместимости, если потребуется чип E32
 #endif
 
 };
 
 #endif // RADIO_MANAGER_H