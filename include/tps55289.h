/*!
    \file
    \brief Класс для работы с Buck-Boost преобразователем TPS55289 через I2C интерфейс
    \authors Близнец Р.А (r.bliznets@gmail.com).
    \version 1.0.0.0
    \date 05.02.2025
*/
#pragma once

#include "driver/i2c_master.h" ///< Заголовочный файл для работы с I2C (ESP-IDF)

/// I2C адрес микросхемы TPS55289 (7-битный формат)
#define TPS55289_I2C_ADDR (0x75)

/*!
    \brief Класс для управления Buck-Boost преобразователем TPS55289
    \details Обеспечивает базовые операции: настройка напряжения, ограничение тока,
             чтение статуса через I2C интерфейс
*/
class TPS55289
{
private:
    int16_t m_i2c_num;                ///< Номер используемого I2C-порта (0, 1 и т.д.)
    gpio_num_t mEnable = GPIO_NUM_NC; ///< Пин управления питанием (GPIO_NUM_NC - не используется)

protected:
    i2c_master_dev_handle_t mI2CDevice; ///< Хэндл подключенного I2C устройства
    float mVoltage = 0.0;               ///< Текущее выходное напряжение (в вольтах)
    float mCurrentLimit = 5.0;          ///< Лимит выходного тока (в амперах)

    /// @brief Запись значения в регистр преобразователя
    /// @param addr Адрес регистра (0x00-0xFF)
    /// @param value 8-битное значение для записи
    /// @return Код ошибки ESP-IDF
    esp_err_t writeReg(uint8_t addr, uint8_t value);

    /// @brief Чтение значения из регистра преобразователя
    /// @param addr Адрес регистра (0x00-0xFF)
    /// @param value Указатель для сохранения прочитанного значения
    /// @return Код ошибки ESP-IDF
    esp_err_t readReg(uint8_t addr, uint8_t *value);

    /// @brief Аварийное отключение преобразователя
    /// @details Сбрасывает настройки и отключает выход
    void abort();

public:
    /// @brief Проверка наличия устройства на шине I2C
    /// @param i2c_num Номер I2C-порта
    /// @param pwr_pin Пин питания (если требуется активация по питанию)
    /// @return true - устройство найдено, false - отсутствует ответ
    static bool i2cProbe(int16_t i2c_num, int16_t pwr_pin = -1);

    /// @brief Конструктор преобразователя
    /// @param i2c_num Номер I2C-порта (0, 1 и т.д.)
    /// @param pwr_pin Пин управления питанием (GPIO_NUM_NC - не используется)
    TPS55289(int16_t i2c_num, int16_t pwr_pin = -1);

    /// @brief Деструктор (автоматически отключает устройство)
    ~TPS55289();

    /// @brief Установка выходного напряжения
    /// @param volts Целевое напряжение (0.8-22.0 В). Если 0 отключает.
    /// @return true - успешная установка, false - ошибка
    bool setVoltage(float volts = 0.0);

    /// @brief Получение текущего напряжения
    /// @return Последнее успешно установленное напряжение
    inline float getVoltage() { return mVoltage; };

    /// @brief Установка лимита выходного тока
    /// @param ampers Значение тока (0.0-8.0 А)
    /// @return true - успешная установка, false - ошибка
    bool setCurrentLimit(float ampers);

    /// @brief Получение текущего лимита тока
    /// @return Последний успешно установленный лимит тока
    inline float getCurrentLimit() { return mCurrentLimit; };

    /// @brief Чтение регистра статуса
    /// @return 8-битное значение регистра статуса
    /// @details Битовая маска:
    ///          [7] - Short circuit protection. 1b = Short circuit happens.
    ///          [6] - Overcurrent protection. 1b = Output current hits the current limit
    ///          [5] - Overvoltage protection. 1b = Output voltage exceeds the OVP threshold.
    ///          [1-0] - 00b = Boost, 01b = Buck, 10b = Buck-Boost, 11b = Reserved
    uint8_t getStatus();
};