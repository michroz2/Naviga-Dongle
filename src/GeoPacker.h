#ifndef GEOPACKER_H
#define GEOPACKER_H

#include <stdint.h>

/**
 * Класс для компактной упаковки географических координат в 32-битное целое число.
 * Использует 16 бит для широты и 16 бит для долготы.
 */
class GeoPacker {
public:
    /**
     * Конструктор инициализирует множитель долготы значением по умолчанию (2.0).
     */
    GeoPacker();

    /**
     * Вычисляет множитель долготы на основе широты местности.
     * Округляет значение до сотых для обеспечения синхронизации между устройствами.
     * @param currentLat Текущая широта в градусах.
     */
    void updateLonScale(float currentLat);

    /**
     * Упаковывает float координаты в uint32_t.
     * @param lat Широта.
     * @param lon Долгота.
     * @return Упакованное 32-битное значение.
     */
    uint32_t pack(float lat, float lon) const;

    /**
     * Восстанавливает полные координаты из 16-битных сегментов относительно опорной точки.
     * @param packed Упакованное 32-битное значение.
     * @param myLat Широта опорной точки (текущего устройства).
     * @param myLon Долгота опорной точки (текущего устройства).
     * @param outLat Ссылка для записи восстановленной широты.
     * @param outLon Ссылка для записи восстановленной долготы.
     */
    void unpack(uint32_t packed, float myLat, float myLon, 
                float &outLat, float &outLon) const;

    /**
     * Возвращает текущий множитель долготы.
     */
    float getLonScale() const;

private:
    float lonExtraScale; // Адаптивный множитель для долготы
    static constexpr float COORD_SCALE = 100000.0f; // Базовый множитель (5 знаков)

    /**
     * Внутренний метод для восстановления 32-битной координаты из 16-битного фрагмента.
     */
    int32_t recoverComponent(int32_t referenceFull, uint16_t received16) const;
}; // Конец объявления класса GeoPacker

#endif // GEOPACKER_H