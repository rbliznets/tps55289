/*!
    \file
    \brief Класс конвертора (TPS55289 30-V, 8-A Buck-Boost Converter with I2C Interface).
    \authors Близнец Р.А. (r.bliznets@gmail.com).
    \version 1.0.0.0
    \date 05.02.2025
*/

#include "tps55289.h"
#include "I2CMaster.h"
#include "CTrace.h"
#include "driver/gpio.h"

static const char *TAG = "TPS55289"; ///< Тег для логирования.

/// Проверка наличия устройства TPS55289 на шине I2C.
/*!
 * \param i2c_num Номер I2C-шины.
 * \param pwr_pin Номер пина управления питанием.
 * \return true, если устройство найдено, иначе false.
 */
bool TPS55289::i2cProbe(int16_t i2c_num, int16_t pwr_pin)
{
    bool res = false;
    gpio_config_t io_conf;
    if (pwr_pin >= 0)
    {
        io_conf = {
            .pin_bit_mask = BIT64(pwr_pin),        ///< Маска пина.
            .mode = GPIO_MODE_OUTPUT,              ///< Режим пина (выход).
            .pull_up_en = GPIO_PULLUP_ENABLE,      ///< Включение подтягивающего резистора.
            .pull_down_en = GPIO_PULLDOWN_DISABLE, ///< Выключение опускающего резистора.
            .intr_type = GPIO_INTR_DISABLE};       ///< Отключение прерываний.

        gpio_config(&io_conf);                  ///< Настройка пина.
        gpio_set_level((gpio_num_t)pwr_pin, 1); ///< Включение питания.
        vTaskDelay(100);                        ///< Задержка для стабилизации питания.
    }

    if (I2CMaster::Instance()->probe(i2c_num, TPS55289_I2C_ADDR)) ///< Проверка наличия устройства.
    {
        res = true;
    }

    if (pwr_pin >= 0)
    {
        gpio_set_level((gpio_num_t)pwr_pin, 0);   ///< Выключение питания.
        io_conf.mode = GPIO_MODE_DISABLE;         ///< Отключение пина.
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE; ///< Выключение подтягивающего резистора.
        gpio_config(&io_conf);                    ///< Обновление настроек пина.
    }

    return res; ///< Возврат результата проверки.
}

/// Конструктор.
/*!
 * Инициализация конвертора TPS55289.
 * \param i2c_num Номер I2C-шины.
 * \param pwr_pin Номер пина управления питанием.
 * \param failed callback на отказ устройства
 */
TPS55289::TPS55289(int16_t i2c_num, int16_t pwr_pin, onTpsFailed *failed) : m_i2c_num(i2c_num), mEnable((gpio_num_t)pwr_pin), mOnFailed(failed)
{
    I2CMaster::Instance()->take(m_i2c_num); ///< Захват I2C-шины.

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, ///< Длина адреса устройства (7 бит).
        .device_address = TPS55289_I2C_ADDR,   ///< Адрес устройства.
        .scl_speed_hz = 400000,                ///< Скорость шины SCL (400 кГц).
        .scl_wait_us = 0,                      ///< Время ожидания.
        .flags = {0}};                         ///< Флаги.

    if (!I2CMaster::Instance()->add(m_i2c_num, &dev_cfg, &mI2CDevice)) ///< Добавление устройства на шину I2C.
    {
        ESP_LOGE(TAG, "Add i2c failed"); ///< Логирование ошибки.
        if (mOnFailed != nullptr)
            mOnFailed(this);
        return;
    }

    if (mEnable >= 0)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = BIT64(pwr_pin),        ///< Маска пина.
            .mode = GPIO_MODE_OUTPUT,              ///< Режим пина (выход).
            .pull_up_en = GPIO_PULLUP_ENABLE,      ///< Включение подтягивающего резистора.
            .pull_down_en = GPIO_PULLDOWN_DISABLE, ///< Выключение опускающего резистора.
            .intr_type = GPIO_INTR_DISABLE};       ///< Отключение прерываний.

        gpio_config(&io_conf);      ///< Настройка пина.
        gpio_set_level(mEnable, 0); ///< Выключение питания.
        vTaskDelay(10);             ///< Задержка.
    }
}

/// Деструктор.
/*!
 * Освобождение ресурсов и удаление устройства с шины I2C.
 */
TPS55289::~TPS55289()
{
    esp_err_t ret;
    setVoltage(0.0); ///< Установка напряжения на 0 В.
    if (mEnable >= 0)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = BIT64(mEnable),        ///< Маска пина.
            .mode = GPIO_MODE_DISABLE,             ///< Отключение пина.
            .pull_up_en = GPIO_PULLUP_DISABLE,     ///< Выключение подтягивающего резистора.
            .pull_down_en = GPIO_PULLDOWN_DISABLE, ///< Выключение опускающего резистора.
            .intr_type = GPIO_INTR_DISABLE};       ///< Отключение прерываний.

        gpio_config(&io_conf); ///< Обновление настроек пина.
    }
    ret = i2c_master_bus_rm_device(mI2CDevice); ///< Удаление устройства с шины I2C.
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Device deleting failed: %d", ret); ///< Логирование ошибки.
    I2CMaster::Instance()->release(m_i2c_num);            ///< Освобождение I2C-шины.
}

/// Запись значения в регистр устройства.
/*!
 * \param addr Адрес регистра.
 * \param value Значение для записи.
 * \return Код ошибки ESP.
 */
esp_err_t TPS55289::writeReg(uint8_t addr, uint8_t value)
{
    uint8_t write_buf[2] = {addr, value};                                                ///< Буфер для записи.
    esp_err_t res = i2c_master_transmit(mI2CDevice, write_buf, 2, TPS55289_I2C_TIMEOUT); ///< Запись данных в регистр.
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Write error 0x%02x:%d", addr, value);
        if (mOnFailed != nullptr)
            mOnFailed(this);
    }
    else
    {
        ESP_LOGD(TAG, "Write regs 0x%02x:%d", addr, value);
    }
    return res;
}

/// Чтение значения из регистра устройства.
/*!
 * \param addr Адрес регистра.
 * \param value Указатель на переменную для хранения прочитанного значения.
 * \return Код ошибки ESP.
 */
esp_err_t TPS55289::readReg(uint8_t addr, uint8_t *value)
{
    esp_err_t res = i2c_master_transmit_receive(mI2CDevice, &addr, 1, value, 1, TPS55289_I2C_TIMEOUT); ///< Чтение данных из регистра.
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Read error 0x%02x", addr);
        if (mOnFailed != nullptr)
            mOnFailed(this);
    }
    else
    {
        ESP_LOGD(TAG, "Read regs 0x%02x:%d", addr, *value);
    }
    return res;
}

/// Прерывание работы конвертора.
void TPS55289::abort()
{
    if (mEnable >= 0)
        gpio_set_level(mEnable, 0); ///< Выключение питания.
    ESP_LOGE(TAG, "abort");         ///< Логирование прерывания.
    mVoltage = 0.0;                 ///< Сброс напряжения.
}

/// Установка напряжения на выходе конвертора.
/*!
 * \param volts Напряжение (0.8 ... 22.0 В).
 * \return true, если напряжение успешно установлено, иначе false.
 */
bool TPS55289::setVoltage(float volts)
{
    if (mVoltage == volts)
        return true; ///< Возврат, если напряжение уже установлено.

    if (((volts < 0.8) && (volts != 0.0)) || (volts > 22)) ///< Проверка допустимого диапазона напряжения.
    {
        ESP_LOGW(TAG, "setVoltage 0.8 .. 22.0: %f failed", volts); ///< Логирование ошибки.
        return false;
    }

    if (volts == 0.0) ///< Установка напряжения на 0 В.
    {
        if (writeReg(6, 0x30) != ESP_OK) ///< Отключение выхода.
        {
            abort(); ///< Прерывание работы.
            return false;
        }
        vTaskDelay(100); ///< Задержка.
        mVoltage = 0.0;  ///< Сброс напряжения.
        if (mEnable >= 0)
            gpio_set_level(mEnable, 0); ///< Выключение питания.
        ESP_LOGI(TAG, "off");           ///< Логирование отключения.
        return true;
    }

    if (mVoltage != 0) ///< Если напряжение уже установлено, отключаем выход.
    {
        if (writeReg(6, 0) != ESP_OK)
        {
            abort(); ///< Прерывание работы.
            return false;
        }
    }
    else ///< Если напряжение не установлено, включаем питание и устанавливаем предельный ток.
    {
        if (mEnable >= 0)
            gpio_set_level(mEnable, 1);             ///< Включение питания.
        vTaskDelay(10);                             ///< Задержка.
        uint8_t r2 = (uint8_t)(mCurrentLimit * 20); ///< Вычисление значения предельного тока.
        if (writeReg(2, (r2 | 0x80)) != ESP_OK)     ///< Установка предельного тока.
        {
            return false;
        }
    }

    uint16_t ref = (uint16_t)(volts * 100 - 79); ///< Вычисление значения напряжения.
    if (writeReg(0, (uint8_t)ref) != ESP_OK)     ///< Запись младшего байта значения напряжения.
    {
        abort(); ///< Прерывание работы.
        return false;
    }
    if (writeReg(1, (uint8_t)(ref >> 8)) != ESP_OK) ///< Запись старшего байта значения напряжения.
    {
        abort(); ///< Прерывание работы.
        return false;
    }
    if (writeReg(6, 162) != ESP_OK) ///< Включение выхода.
    {
        abort(); ///< Прерывание работы.
        return false;
    }

    mVoltage = volts;                      ///< Обновление текущего напряжения.
    ESP_LOGI(TAG, "setVoltage %f", volts); ///< Логирование установки напряжения.
    return true;
}

/// Установка предельного тока на выходе конвертора.
/*!
 * \param ampers Ток (0.01 ... 8.0 А).
 * \return true, если ток успешно установлен, иначе false.
 */
bool TPS55289::setCurrentLimit(float ampers)
{
    if (mCurrentLimit == ampers)
        return true; ///< Возврат, если ток уже установлен.

    if (((ampers < 0.01) && (ampers != 0.0)) || (ampers > 8)) ///< Проверка допустимого диапазона тока.
    {
        ESP_LOGW(TAG, "setCurrentLimit 0.01 .. 8.0: %f failed", ampers); ///< Логирование ошибки.
        return false;
    }

    if (mVoltage == 0.0) ///< Если напряжение не установлено, обновляем значение тока.
    {
        mCurrentLimit = ampers;
        return true;
    }

    if (ampers == 0.0) ///< Установка тока на 0 А.
    {
        if (writeReg(2, 0x64) != ESP_OK) ///< Установка предельного тока на 0 А.
        {
            return false;
        }
        mCurrentLimit = ampers; ///< Обновление текущего тока.
        return true;
    }

    uint8_t r2 = (uint8_t)(ampers * 20);    ///< Вычисление значения предельного тока.
    if (writeReg(2, (r2 | 0x80)) != ESP_OK) ///< Установка предельного тока.
    {
        return false;
    }

    mCurrentLimit = ampers; ///< Обновление текущего тока.
    return true;
}

/// Получение статуса устройства.
/*!
 * \return Статус устройства.
 */
uint8_t TPS55289::getStatus()
{
    if (mVoltage == 0.0)
        return 0xff; ///< Возврат 0xff, если напряжение не установлено.

    uint8_t status = 0xff;             ///< Инициализация статуса.
    if (readReg(7, &status) != ESP_OK) ///< Чтение статуса.
    {
        ESP_LOGW(TAG, "getStatus failed"); ///< Логирование ошибки.
    }
    return status; ///< Возврат статуса.
}