/**
 * File: BleConfig.h
 * Version: 1.26
 * Description: UUID сервиса и характеристик для связи со смартфоном по Bluetooth (UC-02).
 */
 #ifndef BLE_CONFIG_H
 #define BLE_CONFIG_H
 
 // UUID Сервиса, по которому мобильное приложение (Оператор) находит Донгл
 #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
 
 // UUID Характеристики RX (Прием от телефона). Свойства: WRITE
 #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
 
 // UUID Характеристики TX (Отправка в телефон). Свойства: NOTIFY
 #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
  
 // Базовое имя, которое будет расширяться MAC-суффиксом в BleManager
 #define BLE_DEVICE_NAME        "Naviga-Dongle"
 
 #endif