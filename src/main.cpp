/**
 * Project: Naviga-Dongle (T-Beam v1.1 Custom E22 + Universal GPS)
 * File: main.cpp
 * Description: Система логирования GPS координат, умная инициализация GPS
 * и обмен данными через LoRa (SX1268). Основной файл программы.
 */

 #include <Arduino.h>
 #include <Wire.h>
 #include <SPI.h>              
 #include <RadioLib.h>         
 #include <XPowersLib.h>       
 #include <TinyGPS++.h>
 #include "SSD1306Wire.h"
 #include "configuration.h"
 #include "logger.h"           
 
 // --- ОБЪЕКТЫ УПРАВЛЕНИЯ ПЕРИФЕРИЕЙ ---
 XPowersAXP2101 pmu;                                    // Объект для управления контроллером питания (PMU)
 SSD1306Wire display(0x3c, I2C_SDA, I2C_SCL);           // Объект управления OLED дисплеем по шине I2C (адрес 0x3C)
 TinyGPSPlus gps;                                       // Объект парсера NMEA строк для обработки данных GPS
 HardwareSerial GPS_Serial(1);                          // Объпаратный UART порт (UART1) для связи с GPS модулем
 
 // Инициализация радиомодуля SX1268 с указанием пинов управления из configuration.h
 SX1268 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
 
 // --- ПЕРЕМЕННЫЕ И ОБРАБОТЧИКИ ПРЕРЫВАНИЙ ---
 volatile bool receivedFlag = false;                    // Флаг, устанавливаемый в прерывании при успешном получении пакета LoRa

// Макрос для размещения функции прерывания в быстрой памяти (IRAM) для плат ESP
 #if defined(ESP8266) || defined(ESP32)
   ICACHE_RAM_ATTR
 #endif
 /**
  * @brief Обработчик аппаратного прерывания от радиомодуля.
  * Вызывается автоматически при срабатывании пина DIO1 (завершение приема пакета).
  */
 void setFlag(void) {
     receivedFlag = true;
 } // setFlag()

 // Глобальные переменные навигационных данных
 float dist = 0.0;                                      // Дистанция (в метрах) от текущей точки до удаленного объекта
 float azmt = 0.0;                                      // Азимут (в градусах) от текущей точки на удаленный объект
 float remoteLat = 0.0;                                 // Широта, полученная по радиоканалу
 float remoteLon = 0.0;                                 // Долгота, полученная по радиоканалу

 // --- ТАЙМЕРЫ УПРАВЛЕНИЯ ПОТОКАМИ (ПЛАНИРОВЩИК) ---
 uint32_t lastTxTime = 0;                               // Время (в мс) последней отправки координат в эфир
 uint32_t lastGpsLogTime = 0;                           // Время (в мс) последнего обновления дисплея и логов
 uint32_t lastRxTime = 0;                               // Время (в мс) последнего успешного приема чужих координат

 // --- ФУНКЦИИ ИНТЕРФЕЙСА ---

 /**
  * @brief Выводит стандартизированное 3-строчное текстовое меню на OLED дисплей.
  * @param line1 Текст для верхней строки (координата y=0)
  * @param line2 Текст для средней строки (координата y=22)
  * @param line3 Текст для нижней строки (координата y=44)
  */
 void showStatus(String line1, String line2, String line3) {
     display.clear();
     display.setFont(ArialMT_Plain_16);
     display.drawString(0, 0,  line1);
     display.drawString(0, 22, line2);
     display.drawString(0, 44, line3);
     display.display();
 } // showStatus()

 /**
  * @brief Отображает стартовый логотип и системное сообщение при включении устройства.
  * Блокирует выполнение на 2 секунды для чтения пользователем.
  */
 void showLogo() {
     display.clear();
     display.setFont(ArialMT_Plain_16);
     display.drawString(0, 0,  "Naviga-Dongle");
     display.drawString(0, 22, "System Init...");
     display.drawString(0, 44, "Please Wait");
     display.display();
     delay(2000); // Держим логотип на экране 2 секунды
 } // showLogo()

 // --- GPS INIT LOGIC ---

 // Массив стандартных скоростей (baud rates) для автоопределения и сканирования GPS модулей
 const uint32_t baudRates[] = {9600, 115200, 38400, 57600, 19200, 4800};
 const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]); // Вычисление количества элементов в массиве скоростей
 uint32_t originalBaud = 0; // Переменная для хранения скорости, на которой модуль был обнаружен первоначально

 // Бинарная команда протокола UBX-CFG-CFG (Factory Reset). 
 // Сбрасывает настройки u-blox модулей к заводским значениям.
 const uint8_t UBX_FACTORY_RESET[] = {
     0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 
     0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
     0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A
 };

 /**
  * @brief Проверяет наличие корректного текстового NMEA-потока на заданной скорости UART.
  * @param baud Скорость порта (baud rate) для проверки.
  * @return true, если за 1.5 секунды найдена валидная NMEA последовательность ('$G' или '$P').
  */
 bool checkNMEA(uint32_t baud) {
     GPS_Serial.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
     LOG_INFO("GPS", "Checking %d baud...", baud);
     
     unsigned long start = millis();
     char prevChar = 0; // Хранение предыдущего символа для поиска связки
     
     // Ожидание данных в течение 1.5 секунд (1500 мс)
     while (millis() - start < 1500) {
         if (GPS_Serial.available()) {
             char c = GPS_Serial.read();
             // Поиск стартового маркера '$' за которым следует 'G' (напр. $GNRMC) или 'P' (напр. $PUBX)
             if (prevChar == '$' && (c == 'G' || c == 'P')) {
                 LOG_INFO("GPS", "OK (Valid NMEA Found!)");
                 return true;
             } // if (prevChar == '$'...)
             prevChar = c;
         } // if (GPS_Serial.available())
     } // while (timeout)
     LOG_WARN("GPS", "Fail");
     return false;
 } // checkNMEA()

 /**
  * @brief Универсальная функция инициализации GPS.
  * Производит сканирование скоростей, находит модуль, переключает его на 9600 бод (если нужно)
  * или запускает алгоритм жесткого восстановления (Rescue Mode), если модуль завис в бинарном режиме.
  */
 void initGPS() {
     showStatus("Init GPS...", "Searching module", "Wait...");

     bool nmeaFound = false;
     uint32_t activeBaud = 0;

     // Шаг 1: Сканирование всех заданных скоростей на наличие NMEA данных
     for (int i = 0; i < numBauds; i++) {
         if (checkNMEA(baudRates[i])) {
             activeBaud = baudRates[i];
             originalBaud = activeBaud; // Запоминаем изначальную скорость модуля
             nmeaFound = true;
             break;
         } // if (checkNMEA)
     } // for (baud rates)

     // Шаг 2: Модуль найден. Обработка текущей скорости.
     if (nmeaFound) {
         if (activeBaud == 9600) {
             // Модуль уже работает на целевой скорости, никаких действий не требуется
             LOG_INFO("GPS", "GPS is ready at 9600 baud.");
             return; 
         } else {
             // Модуль работает на другой скорости. Попытка программного переключения на 9600.
             showStatus("Init GPS...", "Switching Baud", String(activeBaud) + " -> 9600");
             LOG_INFO("GPS", "Sending $PUBX command at %d baud to switch to 9600...", activeBaud);
             
             // Отправка проприетарной NMEA команды (u-blox) на смену скорости порта
             GPS_Serial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n");
             GPS_Serial.flush();
             delay(500); // Ожидание применения новых настроек UART модулем
             
             LOG_INFO("GPS", "Verifying switch to 9600...");
             // Проверка успешности переключения скорости
             if (checkNMEA(9600)) {
                 LOG_INFO("GPS", "Successfully switched to 9600!");
                 return; 
             } else {
                 LOG_WARN("GPS", "Switch verification FAILED. Escalating to Rescue Mode.");
                 nmeaFound = false; // Принудительный сброс флага для перехода к аппаратному лечению
             } // if (checkNMEA(9600)) else
         } // if (activeBaud == 9600) else
     } // if (nmeaFound)

     // Шаг 3: Rescue Mode. Выполняется, если модуль не найден или не поддается переключению
     if (!nmeaFound) {
         showStatus("Init GPS...", "Rescue Mode!", "Wait 10 sec...");
         LOG_INFO("GPS", "ENTERING GPS RESCUE MODE");

         // Агрессивная рассылка бинарной команды сброса на всех возможных скоростях UART
         for (int i = 0; i < numBauds; i++) {
             GPS_Serial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
             delay(50);
             for(int j = 0; j < 3; j++) {
                 GPS_Serial.write(UBX_FACTORY_RESET, sizeof(UBX_FACTORY_RESET));
                 GPS_Serial.flush();
                 delay(50);
             } // for (j < 3)
         } // for (baud rates)

         // Аппаратная перезагрузка: отключение и включение линии питания ядра GPS (ALDO3)
         LOG_INFO("GPS", "Power cycling GPS...");
         pmu.disableALDO3();
         delay(2000); // Пауза для разряда конденсаторов модуля
         pmu.enableALDO3();
         delay(2000); // Пауза для загрузки микропрограммы GPS после старта питания

         LOG_INFO("GPS", "Re-evaluating after Rescue...");
         // Повторная проверка стандартных скоростей после сброса
         if (checkNMEA(115200)) {
             originalBaud = 115200;
             LOG_INFO("GPS", "Rescued at 115200. Sending switch command...");
             // Если модуль ожил на 115200 (типично для сброшенного M8N), переводим его на 9600
             GPS_Serial.print("$PUBX,41,1,0007,0003,9600,0*10\r\n");
             GPS_Serial.flush();
             delay(500);
         } else if (checkNMEA(9600)) {
             // Модуль ожил на целевой скорости (типично для NEO-6M)
             originalBaud = 9600;
         } else {
             // Критическая аппаратная неисправность GPS модуля
             originalBaud = 0;
         } // if-else if-else
         
         // Финальная настройка UART контроллера ESP32 на рабочую скорость
         GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
         LOG_INFO("GPS", "GPS Init Protocol Finished. Listening at 9600.");
     } // if (!nmeaFound)
 } // initGPS()

 // --- LORA INIT LOGIC ---

 /**
  * @brief Инициализация и настройка параметров радиомодуля SX1268 (LoRa).
  */
 void initLoRa() {
     showStatus("System Init...", "GPS Init Done", "Init LoRa...");
     LOG_INFO("LORA", "Initializing SX1268...");
     
     // Запуск модуля на частоте 433.0 МГц
     int state = radio.begin(433.0);        
 
     if (state == RADIOLIB_ERR_NONE) {
         // Применение оптимальных параметров модуляции LoRa для нашей задачи
         radio.setDio2AsRfSwitch(false);    // Отключение управления внутренним RF-переключателем через DIO2 (используем внешний)
         radio.setBandwidth(125.0);         // Ширина полосы пропускания (125 кГц)
         radio.setSpreadingFactor(9);       // Фактор расширения (SF9) - компромисс между дальностью и скоростью
         radio.setCodingRate(5);            // Скорость кодирования (CR 4/5) для коррекции ошибок
         radio.setSyncWord(0x2B);           // Установка слова синхронизации для приватной сети
         radio.setPreambleLength(16);       // Длина преамбулы (16 символов)
         radio.setOutputPower(22);          // Мощность передатчика (22 дБм)
         radio.setCurrentLimit(140);        // Ограничение потребляемого тока (140 мА) для защиты питания
 
         // Настройка напряжения питания встроенного термокомпенсированного кварца (TCXO)
         if (radio.setTCXO(1.8) != RADIOLIB_ERR_NONE) {
              LOG_ERROR("LORA", "TCXO setup failed!");
         } // if (TCXO fail)
 
         // Назначение пинов управления внешним антенным переключателем (RX/TX Enable)
         radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
         
         // Включение режима повышенного усиления приемника
         state = radio.setRxBoostedGainMode(true);
         if (state != RADIOLIB_ERR_NONE) {
             LOG_ERROR("LORA", "RX Boost setup failed!");
         } // if (RX Boost fail)

         // Привязка функции-обработчика (ISR) к событию получения пакета
         radio.setPacketReceivedAction(setFlag);
         
         // Запуск модуля в режим непрерывного прослушивания эфира (RX Mode)
         state = radio.startReceive();
         if (state == RADIOLIB_ERR_NONE) {
             LOG_INFO("LORA", "Reception STARTED");
         } else {
             LOG_ERROR("LORA", "Failed to start reception, code: %d", state);
         } // if (startReceive) else
     } else {
         // Радиомодуль не ответил по шине SPI при инициализации
         LOG_ERROR("LORA", "Radio init failed, code: %d", state);
         showStatus("ERROR", "LoRa Init Failed", "Check Logs");
         delay(3000);
     } // if (state == RADIOLIB_ERR_NONE) else
 } // initLoRa()

 // --- ФУНКЦИЯ ПЕРЕДАЧИ КООРДИНАТ ---

 /**
  * @brief Упаковывает текущие валидные GPS координаты в бинарный пакет и отправляет их в эфир.
  */
 void sendLocation() {
     // Защита от отправки "нулевых" координат до получения валидного FIX-а от GPS
     if (!gps.location.isValid()) {
         LOG_WARN("TX", "Skip TX: GPS location not valid.");
         return;
     } // if (!isValid)

     // Конвертация координат из float (градусы) в int32_t с сохранением 5 знаков после запятой
     int32_t latInt = (int32_t)(gps.location.lat() * 100000);
     int32_t lonInt = (int32_t)(gps.location.lng() * 100000);

     // Создание и заполнение 8-байтного буфера полезной нагрузки (Payload)
     uint8_t txBuffer[8];
     memcpy(txBuffer, &latInt, 4);
     memcpy(txBuffer + 4, &lonInt, 4);

     LOG_INFO("TX", "Starting transmission... (Lat: %d, Lon: %d)", latInt, lonInt);

     // Перевод модуля в режим Standby перед началом передачи (обязательное требование SX126x)
     radio.standby();
     int state = radio.transmit(txBuffer, 8); // Блокирующая функция передачи пакета

     if (state == RADIOLIB_ERR_NONE) {
         LOG_INFO("TX", "Transmission finished successfully.");
     } else {
         LOG_ERROR("TX", "Transmission failed, code: %d", state);
     } // if (transmit state) else

     // ХИРУРГИЧЕСКОЕ ВМЕШАТЕЛЬСТВО: 
     // Очистка флага прерывания, который гарантированно взвелся событием "TxDone" (окончание передачи)
     receivedFlag = false;

     // Возврат модуля в режим прослушивания эфира после окончания передачи
     radio.startReceive();
 } // sendLocation()
 
 void setup() {
     delay(500); 
     Serial.begin(115200);
     // Ожидание готовности Serial монитора (с таймаутом 3 секунды)
     unsigned long start = millis();
     while (!Serial && (millis() - start < 3000));
     
     LOG_INFO("SYS", "--- DONGLE BOOT START ---");
 
     // 0. ИЗОЛЯЦИЯ ВСТРОЕННОГО МОДУЛЯ (Отключение мешающего модуля от шины SPI)
     pinMode(LORA_ONBOARD_CS, OUTPUT);
     digitalWrite(LORA_ONBOARD_CS, HIGH);
 
     // Инициализация пина системного светодиода и его выключение
     pinMode(LED_PIN, OUTPUT);
     digitalWrite(LED_PIN, LED_OFF); // IO4 low level turns LED off on T-Beam 
 
     // 1. ИНИЦИАЛИЗАЦИЯ ШИН ДАННЫХ
     Wire.begin(I2C_SDA, I2C_SCL);                       // Запуск I2C (для дисплея и PMU)
     SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); // Запуск SPI (для LoRa)
 
     // 2. КОНФИГУРАЦИЯ ПИТАНИЯ (AXP2101)
     LOG_INFO("PMU", "Initializing AXP2101...");
     bool pmuFound = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
     
     if (pmuFound) {
         pmu.setALDO2Voltage(3300);         // Включение питания VCC для внешнего LoRa модуля (3.3V)
         pmu.enableALDO2();
         pmu.setALDO3Voltage(3300);         // Включение питания для ядра GPS модуля (3.3V)
         pmu.enableALDO3();
         pmu.disableALDO4();                // Отключение неиспользуемого канала для экономии энергии
         pmu.enableSystemVoltageMeasure();  // Активация внутреннего АЦП для измерения напряжения батареи
     } // if (pmuFound)
 
     // 3. ПОДГОТОВКА ИНТЕРФЕЙСА
     display.init();
     display.flipScreenVertically();
     showLogo();
 
     // 4. УМНАЯ ИНИЦИАЛИЗАЦИЯ GPS
     initGPS();
 
     // 5. КОНФИГУРАЦИЯ RADIOLIB (SX1268)
     initLoRa();
 
     // 6. ИНДИКАЦИЯ ЗАВЕРШЕНИЯ ИНИЦИАЛИЗАЦИИ
     LOG_INFO("SYS", "Setup completed successfully! System is READY.");
 } // setup()
 
 void loop() {
     // 1. НЕПРЕРЫВНОЕ ЧТЕНИЕ ДАННЫХ GPS
     // Парсер TinyGPS++ требует постоянной передачи байтов из UART буфера
     while (GPS_Serial.available() > 0) {
         gps.encode(GPS_Serial.read());
     } // while (GPS_Serial)

     // 2. ОБРАБОТКА ПРИНЯТЫХ ПАКЕТОВ LORA (Управляемая прерываниями)
     if (receivedFlag) {
         // Критическая секция: отключение прерываний при сбросе флага во избежание состояния гонки
         noInterrupts();                    
         receivedFlag = false;
         interrupts();

         // Получение длины принятого пакета
         size_t len = radio.getPacketLength();

         // ЗАЩИТНАЯ БРОНЯ: Фильтрация фантомных пакетов (ложных прерываний от шума)
         if (len > 0) {
             uint8_t rxBuffer[256];             
             int state = radio.readData(rxBuffer, len); // Чтение данных из FIFO буфера радиомодуля

             if (state == RADIOLIB_ERR_NONE) {
                 LOG_INFO("LORA", "Packet Received! Length: %d bytes", len);
                 
                 // Если длина совпадает с ожидаемой (8 байт = две координаты Int32)
                 if (len == 8) {
                     int32_t latInt, lonInt;
                     memcpy(&latInt, rxBuffer, 4);
                     memcpy(&lonInt, rxBuffer + 4, 4);

                     // Конвертация из Int32 обратно в формат Float (градусы)
                     remoteLat = latInt / 100000.0;
                     remoteLon = lonInt / 100000.0;

                     // Обновление таймера приема для расчета таймаута (30 сек)
                     lastRxTime = millis();

                     LOG_INFO("LORA", "Remote Location Extracted: Lat: %.6f, Lon: %.6f", remoteLat, remoteLon);
                     LOG_INFO("LORA", "RSSI: %.2f dBm | SNR: %.2f dB", radio.getRSSI(), radio.getSNR());
                 } else {
                     // Обработка пакета неизвестной длины/формата (вывод дампа в HEX)
                     String hexStr = "";
                     for (size_t i = 0; i < len; i++) {
                         char buf[4];
                         sprintf(buf, "%02X ", rxBuffer[i]);
                         hexStr += buf;
                     } // for (len)
                     LOG_INFO("LORA", "Unknown Data (HEX): %s", hexStr.c_str());
                 } // if (len == 8) else
                 
             } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                 // Пакет получен, но контрольная сумма не совпадает (повреждение в эфире)
                 LOG_WARN("LORA", "CRC Error!");
             } else {
                 // Аппаратная ошибка при чтении буфера модема
                 LOG_ERROR("LORA", "Reception failed, code: %d", state);
             } // if (readData state) else if else
         } // if (len > 0)
         
         // Перезапуск модуля в режим приема после обработки пакета (или игнорирования фантома)
         radio.startReceive();
     } // if (receivedFlag)

     // 3. ПЛАНИРОВЩИК ПЕРЕДАЧИ (TX Scheduler)
     // Выполняется, если прошло заданное время (txInterval)
     if (millis() - lastTxTime >= txInterval) {
         // CSMA-подобная проверка: не начинать передачу, если в данный момент обрабатывается прием
         if (!receivedFlag) {
             sendLocation();
             lastTxTime = millis();
         } else {
             LOG_WARN("TX", "TX delayed: Air is busy.");
         } // if (!receivedFlag) else
     } // if (txInterval)

     // 4. ПЛАНИРОВЩИК ОБНОВЛЕНИЯ ИНТЕРФЕЙСА И ЛОГИРОВАНИЯ
     // Выполняется каждые gpsUpdateInterval миллисекунд
     if (millis() - lastGpsLogTime >= gpsUpdateInterval) { 
         lastGpsLogTime = millis();
         
         // Системный пульс (Heartbeat): мигание красным светодиодом (пин IO4)
         digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
         
         String gpsStatus;
         String distStr;
         String azmtStr;
         int sats = gps.satellites.value(); // Количество видимых спутников
 
         // НЕЗАВИСИМАЯ ПРОВЕРКА ТАЙМАУТА LORA (выполняется всегда)
         bool isRemoteValid = false;
         if (lastRxTime == 0 || (millis() - lastRxTime > 30000)) {
             // Связь с удаленным устройством потеряна более 30 сек назад (или еще не установлена)
             distStr = "Dist: ???";
             azmtStr = "Azmt: ???";
             LOG_WARN("LORA", "Remote signal timeout (> 30s) or not established");
         } else {
             // Связь активна, чужие координаты актуальны
             isRemoteValid = true;
         } // if (lastRxTime check) else
 
         // ЛОГИКА ОТОБРАЖЕНИЯ СВОЕГО GPS И РАСЧЕТОВ
         if (!gps.location.isValid()) {
             // Наши координаты невалидны (потерян сигнал GPS)
             gpsStatus = (sats > 0) ? ("GPS Wait " + String(sats)) : "GPS ERROR";
             
             // Если мы потеряли свои координаты, мы не можем рассчитать дистанцию, даже если чужие есть
             distStr = "Dist: ***";
             azmtStr = "Azmt: ***";
         } else {
             // Наши координаты валидны
             gpsStatus = "GPS OK " + String(sats);
             LOG_INFO("GPS", "Fix OK! Pos: %.6f, %.6f | Alt: %.1fm", 
                      gps.location.lat(), gps.location.lng(), gps.altitude.meters());
                      
             // Если обе точки валидны - производим расчет
             if (isRemoteValid) {
                 dist = TinyGPSPlus::distanceBetween(
                     gps.location.lat(), gps.location.lng(), 
                     remoteLat, remoteLon
                 );
                 azmt = TinyGPSPlus::courseTo(
                     gps.location.lat(), gps.location.lng(), 
                     remoteLat, remoteLon
                 );
                 
                 distStr = "Dist: " + String((int)dist) + " m";
                 azmtStr = "Azmt: " + String((int)azmt) + " deg";
             } // if (isRemoteValid)
         } // if (!isValid) else
 
         // Обновление информации на OLED дисплее
         showStatus(gpsStatus, distStr, azmtStr);
     } // if (gpsUpdateInterval)
 } // loop()