/**
 * File: BleConfig.h
 * Version: 1.26
 * Изменение: Внедрение UUID для BLE сервиса Naviga (Шаг 4, UC-02).
 * Description: Определение идентификаторов для GATT-сервера.
 */

 #ifndef BLE_CONFIG_H
 #define BLE_CONFIG_H
 
 // --- UUID СЕРВИСА И ХАРАКТЕРИСТИК ---
 
 /**
  * Основной Service UUID для Naviga Dongle.
  * Используется приложением на смартфоне для фильтрации наших устройств в эфире.
  */
 #define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
 
 /**
  * Characteristic RX (Входящая для Донгла).
  * Сюда смартфон записывает (WRITE) данные: настройки, команды, текстовые сообщения.
  */
 #define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
 
 /**
  * Characteristic TX (Исходящая для Донгла).
  * Донгл отправляет сюда данные (NOTIFY): обновленные координаты меш-сети, логи и статус батареи.
  */
 #define CHARACTERISTIC_UUID_TX "82215313-0599-467a-8f4b-0143896541f6"
 
 // --- ПАРАМЕТРЫ РЕКЛАМЫ (ADVERTISING) ---
 #define BLE_DEVICE_NAME        "Naviga-Dongle" // Имя, видимое при сканировании
 #define ADVERTISING_INTERVAL   100             // Интервал рекламы (мс)
 
 #endif // BLE_CONFIG_H