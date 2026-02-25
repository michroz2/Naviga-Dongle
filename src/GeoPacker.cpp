#include "GeoPacker.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GeoPacker::GeoPacker() : lonExtraScale(2.0f) {
} // Конец конструктора

void GeoPacker::updateLonScale(float currentLat) {
    float radians = currentLat * (M_PI / 180.0f);
    float cosLat = cosf(radians);

    // Ограничение для исключения деления на ноль вблизи полюсов
    if (cosLat < 0.01f) {
        cosLat = 0.01f;
    } // Конец проверки cosLat

    float rawScale = 1.0f / cosLat;
    
    // Округление множителя до сотых долей для стабильности между узлами
    lonExtraScale = roundf(rawScale * 100.0f) / 100.0f;
} // Конец метода updateLonScale

uint32_t GeoPacker::pack(float lat, float lon) const {
    int32_t latI = (int32_t)roundf(lat * COORD_SCALE);
    int32_t lonI = (int32_t)roundf(lon * COORD_SCALE * lonExtraScale);

    // Извлечение младших 16 бит для каждого компонента
    uint16_t lat16 = (uint16_t)(latI & 0xFFFF);
    uint16_t lon16 = (uint16_t)(lonI & 0xFFFF);

    // Формирование 32-битного слова: Lat (старшие), Lon (младшие)
    return ((uint32_t)lat16 << 16) | lon16;
} // Конец метода pack

void GeoPacker::unpack(uint32_t packed, float myLat, float myLon, 
                       float &outLat, float &outLon) const {
    uint16_t lat16 = (uint16_t)(packed >> 16);
    uint16_t lon16 = (uint16_t)(packed & 0xFFFF);

    int32_t myLatI = (int32_t)roundf(myLat * COORD_SCALE);
    int32_t myLonI = (int32_t)roundf(myLon * COORD_SCALE * lonExtraScale);

    // Восстановление полных целочисленных значений
    int32_t resLatI = recoverComponent(myLatI, lat16);
    int32_t resLonI = recoverComponent(myLonI, lon16);

    // Обратное преобразование во float
    outLat = (float)resLatI / COORD_SCALE;
    outLon = (float)resLonI / (COORD_SCALE * lonExtraScale);
} // Конец метода unpack

float GeoPacker::getLonScale() const {
    return lonExtraScale;
} // Конец метода getLonScale

int32_t GeoPacker::recoverComponent(int32_t referenceFull, uint16_t received16) const {
    // Совмещение старших бит опорного значения с полученными младшими битами
    int32_t candidate = (referenceFull & 0xFFFF0000) | received16;
    int32_t diff = candidate - referenceFull;

    // Коррекция при пересечении границы 16-битного сегмента (32768 единиц ~ 36 км)
    if (diff > 32768) {
        candidate -= 65536;
    } else if (diff < -32768) {
        candidate += 65536;
    } // Конец блока коррекции границ

    return candidate;
} // Конец метода recoverComponent