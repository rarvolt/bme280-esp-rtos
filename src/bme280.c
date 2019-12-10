//
// Created by rarvolt on 03.12.2019.
//

#include "bme280.h"

#include <stdlib.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include <freertos/task.h>
#include <math.h>


// calibration registers
#define _BME280_REG_DIG_T1      0x88
#define _BME280_REG_DIG_T2      0x8A
#define _BME280_REG_DIG_T3      0x8C
#define _BME280_REG_DIG_P1      0x8E
#define _BME280_REG_DIG_P2      0x90
#define _BME280_REG_DIG_P3      0x92
#define _BME280_REG_DIG_P4      0x94
#define _BME280_REG_DIG_P5      0x96
#define _BME280_REG_DIG_P6      0x98
#define _BME280_REG_DIG_P7      0x9A
#define _BME280_REG_DIG_P8      0x9C
#define _BME280_REG_DIG_P9      0x9E
#define _BME280_REG_DIG_H1      0xA1
#define _BME280_REG_DIG_H2      0xE1
#define _BME280_REG_DIG_H3      0xE3
#define _BME280_REG_DIG_H4      0xE4
#define _BME280_REG_DIG_H5      0xE5
#define _BME280_REG_DIG_H6      0xE6


// register access macros - register address, bitmask
#define _BME280_REG_HUM     0xFD
#define _BME280_REG_TEMP    0xFA
#define _BME280_REG_PRESS   0xF7
#define _BME280_REG_CONFIG   0xF5
#define _BME280_REG_CONTROL 0xF4
#define _BME280_REG_STATUS 0xF3
#define _BME280_REG_CTRL_HUM 0xF2
#define _BME280_REG_RESET       0xE0
#define _BME280_REG_ID          0xD0

// constants
#define _BME280_EXPECTED_ID 0x60
#define _BME280_SOFT_RESET  0xB6

#define BME280_I2C_PORT I2C_NUM_0

const char *BME_TAG = "BME280";

typedef struct
{
    uint16_t dig_T1; ///< temperature compensation value
    int16_t dig_T2;  ///< temperature compensation value
    int16_t dig_T3;  ///< temperature compensation value

    uint16_t dig_P1; ///< pressure compensation value
    int16_t dig_P2;  ///< pressure compensation value
    int16_t dig_P3;  ///< pressure compensation value
    int16_t dig_P4;  ///< pressure compensation value
    int16_t dig_P5;  ///< pressure compensation value
    int16_t dig_P6;  ///< pressure compensation value
    int16_t dig_P7;  ///< pressure compensation value
    int16_t dig_P8;  ///< pressure compensation value
    int16_t dig_P9;  ///< pressure compensation value

    uint8_t dig_H1; ///< humidity compensation value
    int16_t dig_H2; ///< humidity compensation value
    uint8_t dig_H3; ///< humidity compensation value
    int16_t dig_H4; ///< humidity compensation value
    int16_t dig_H5; ///< humidity compensation value
    int8_t dig_H6;  ///< humidity compensation value
} BME280_calib_data_t;


typedef struct
{
    uint8_t t_sb : 3;
    uint8_t filter : 3;
    uint8_t none : 1;
    uint8_t spi3w_en : 1;
} _BME280_reg_config_t;

uint8_t _bme280_reg_config_get(_BME280_reg_config_t *reg)
{
    return (reg->t_sb << 5) | (reg->filter << 2) | reg->spi3w_en;
}

typedef struct
{
    uint8_t osrs_t : 3;
    uint8_t osrs_p : 3;
    uint8_t mode : 2;
} _BME280_reg_meas_t;

uint8_t _bme280_reg_meas_get(_BME280_reg_meas_t *reg)
{
    return (reg->osrs_t << 5) | (reg->osrs_p << 2) | reg->mode;
}

typedef struct
{
    uint8_t none : 5;
    uint8_t osrs_h : 3;
} _BME280_reg_hum;

uint8_t _bme280_reg_hum_get(_BME280_reg_hum *reg)
{
    return reg->osrs_h;
}

struct BME280
{
    BME280_ADDR_t addr;
    BME280_calib_data_t calib_data;
    uint8_t sensor_id;
    int32_t t_fine;
    _BME280_reg_config_t reg_config;
    _BME280_reg_meas_t reg_meas;
    _BME280_reg_hum reg_hum;
};


esp_err_t _bme280_reg_write8(BME280_handle_t bme, uint8_t reg, uint8_t data);
esp_err_t _bme280_reg_read8(BME280_handle_t bme, uint8_t reg, uint8_t *data);
esp_err_t _bme280_reg_read16(BME280_handle_t bme, uint8_t reg, uint16_t *data);
esp_err_t _bme280_reg_read24(BME280_handle_t bme, uint8_t reg, uint32_t *data);
esp_err_t _bme280_reg_readS16(BME280_handle_t bme, uint8_t reg, int16_t *data);
esp_err_t _bme280_reg_read16_LE(BME280_handle_t bme, uint8_t reg, uint16_t *data);
esp_err_t _bme280_reg_readS16_LE(BME280_handle_t bme, uint8_t reg, int16_t *data);

uint8_t _bme280_read_id(BME280_handle_t bme);
bool _bme280_is_reading_calibration(BME280_handle_t bme);
void _bme280_read_calib_data(BME280_handle_t bme, BME280_calib_data_t *data);


BME280_handle_t bme280_init(const BME280_ADDR_t addr, const BME280_config_t *config)
{
    BME280_handle_t bme = malloc(sizeof(struct BME280));
    bme->addr = addr;

    if (_bme280_read_id(bme) != _BME280_EXPECTED_ID)
    {
        ESP_LOGE(BME_TAG, "ID mismatch, probably not BME280");
        return NULL;
    }
    ESP_LOGI(BME_TAG, "Found BME280 device, soft resetting...");
    bme280_soft_reset(bme);
    vTaskDelay(300 / portTICK_RATE_MS);

    while (_bme280_is_reading_calibration(bme))
    {
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    ESP_LOGI(BME_TAG, "Reading calibration data...");
    _bme280_read_calib_data(bme, &bme->calib_data);

    ESP_LOGI(BME_TAG, "Setting configuration");
    bme280_set_config(bme, config);

    vTaskDelay(100 / portTICK_RATE_MS);

    return bme;
}

uint8_t _bme280_read_id(BME280_handle_t bme)
{
    uint8_t data;
    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_ID, &data))
    return data;
}

bool _bme280_is_reading_calibration(BME280_handle_t bme)
{
    uint8_t status;
    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_STATUS, &status))
    return (status & (1 << 0)) != 0;
}

void _bme280_read_calib_data(BME280_handle_t bme, BME280_calib_data_t *data)
{
    uint8_t temp[2];

    ESP_ERROR_CHECK(_bme280_reg_read16_LE(bme, _BME280_REG_DIG_T1, &data->dig_T1))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_T2, &data->dig_T2))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_T3, &data->dig_T3))

    ESP_ERROR_CHECK(_bme280_reg_read16_LE(bme, _BME280_REG_DIG_P1, &data->dig_P1))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_P2, &data->dig_P2))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_P3, &data->dig_P3))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_P4, &data->dig_P4))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_P5, &data->dig_P5))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_P6, &data->dig_P6))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_P7, &data->dig_P7))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_P8, &data->dig_P8))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_P9, &data->dig_P9))

    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_DIG_H1, &data->dig_H1))
    ESP_ERROR_CHECK(_bme280_reg_readS16_LE(bme, _BME280_REG_DIG_H2, &data->dig_H2))
    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_DIG_H3, &data->dig_H3))
    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_DIG_H4, &temp[0]))
    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_DIG_H4 + 1, &temp[1]))
    data->dig_H4 = (temp[0] << 4) | (temp[1] & 0xF);
    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_DIG_H5 + 1, &temp[0]))
    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_DIG_H5, &temp[1]))
    data->dig_H5 = (temp[0] << 4) | (temp[1] >> 4);
    ESP_ERROR_CHECK(_bme280_reg_read8(bme, _BME280_REG_DIG_H6, (uint8_t*)&data->dig_H6))

}

void bme280_set_config(BME280_handle_t bme, const BME280_config_t *config)
{
    bme->reg_config.t_sb = config->t_standby;
    bme->reg_config.filter = config->filter;
    bme->reg_meas.mode = config->mode;
    bme->reg_meas.osrs_t = config->ov_temp;
    bme->reg_meas.osrs_p = config->ov_press;
    bme->reg_hum.osrs_h = config->ov_hum;

    _bme280_reg_write8(bme, _BME280_REG_CONTROL, BME280_MODE_SLEEP);
    _bme280_reg_write8(bme, _BME280_REG_CTRL_HUM, _bme280_reg_hum_get(&bme->reg_hum));
    _bme280_reg_write8(bme, _BME280_REG_CONFIG, _bme280_reg_config_get(&bme->reg_config));
    _bme280_reg_write8(bme, _BME280_REG_CONTROL, _bme280_reg_meas_get(&bme->reg_meas));
}

void bme280_soft_reset(BME280_handle_t bme)
{
    _bme280_reg_write8(bme, _BME280_REG_RESET, _BME280_SOFT_RESET);
}

void bme280_force_measure(BME280_handle_t bme)
{
    uint8_t status;

    if (bme->reg_meas.mode == BME280_MODE_FORCED)
    {
        _bme280_reg_write8(bme, _BME280_REG_CONTROL, _bme280_reg_meas_get(&bme->reg_meas));

        do
        {
            vTaskDelay(5 / portTICK_RATE_MS);
            _bme280_reg_read8(bme, _BME280_REG_STATUS, &status);
        }
        while (status & 0x08);
    }
}

void bme280_read_temperature(BME280_handle_t bme, float *temperature)
{
    float ret;
    int32_t adc_T, var1, var2;

    _bme280_reg_read24(bme, _BME280_REG_TEMP, (uint32_t*)&adc_T);
    if (adc_T == 0x800000)
    {
        ret = NAN;
        return;
    }

    adc_T >>= 4;

    var1 = ((((adc_T >> 3) - ((int32_t)bme->calib_data.dig_T1 << 1))) *
            ((int32_t)bme->calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bme->calib_data.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)bme->calib_data.dig_T1))) >> 12) *
            ((int32_t)bme->calib_data.dig_T3)) >> 14;

    bme->t_fine = var1 + var2;

    ret = (bme->t_fine * 5 + 128) >> 8;

    if (temperature == NULL)
    {
        return;
    }

    *temperature = ret / 100.0f;
}

void bme280_read_pressure(BME280_handle_t bme, float *pressure)
{
    int64_t var1, var2, p;
    int32_t adc_P;

    bme280_read_temperature(bme, NULL);

    _bme280_reg_read24(bme, _BME280_REG_PRESS, (uint32_t*)&adc_P);
    if (adc_P == 0x800000)
    {
        *pressure = NAN;
        return;
    }

    adc_P >>= 4;

    var1 = ((int64_t)bme->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bme->calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bme->calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)bme->calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bme->calib_data.dig_P3) >> 8) +
           ((var1 * (int64_t)bme->calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bme->calib_data.dig_P1) >> 33;

    if (var1 == 0)
    {
        *pressure = 0.0f;
        return;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bme->calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bme->calib_data.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bme->calib_data.dig_P7) << 4);

    *pressure = (float)p / 256;
}

void bme280_read_humidity(BME280_handle_t bme, float *humidity)
{
    int32_t adc_H, v_x1_u32r;

    bme280_read_temperature(bme, NULL);

    _bme280_reg_read16(bme, _BME280_REG_HUM, (uint16_t*)&adc_H);
    if (adc_H == 0x8000)
    {
        *humidity = NAN;
        return;
    }

    v_x1_u32r = (bme->t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme->calib_data.dig_H4) << 20) -
            (((int32_t)bme->calib_data.dig_H5) * v_x1_u32r)) +
            ((int32_t)16384)) >> 15) *
            (((((((v_x1_u32r * ((int32_t)bme->calib_data.dig_H6)) >> 10) *
            (((v_x1_u32r * ((int32_t)bme->calib_data.dig_H3)) >> 11) +
            ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) * ((int32_t)bme->calib_data.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
            ((int32_t)bme->calib_data.dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

    *humidity = (float)(v_x1_u32r >> 12) / 1024.0f;
}

// region I2C functions

#define ACK_CHECK_EN  0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL       0x0
#define NACK_VAL      0x1
#define LAST_NACK_VAL 0x2

#define BME_WRITE(addr) ((addr) << 1 | I2C_MASTER_WRITE)
#define BME_READ(addr) ((addr) << 1 | I2C_MASTER_READ)


esp_err_t _bme280_reg_write8(BME280_handle_t bme, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, BME_WRITE(bme->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(BME280_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}

esp_err_t _bme280_reg_read8(BME280_handle_t bme, uint8_t reg, uint8_t *data)
{
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   ESP_ERROR_CHECK(i2c_master_start(cmd))
   ESP_ERROR_CHECK(i2c_master_write_byte(cmd, BME_WRITE(bme->addr), ACK_CHECK_EN))
   ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN))
   ESP_ERROR_CHECK(i2c_master_stop(cmd))
   ESP_ERROR_CHECK(i2c_master_cmd_begin(BME280_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
   i2c_cmd_link_delete(cmd);

   cmd = i2c_cmd_link_create();
   ESP_ERROR_CHECK(i2c_master_start(cmd))
   ESP_ERROR_CHECK(i2c_master_write_byte(cmd, BME_READ(bme->addr), ACK_CHECK_EN))
   ESP_ERROR_CHECK(i2c_master_read(cmd, data, 1, LAST_NACK_VAL))
   ESP_ERROR_CHECK(i2c_master_stop(cmd))
   ESP_ERROR_CHECK(i2c_master_cmd_begin(BME280_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
   i2c_cmd_link_delete(cmd);

   return ESP_OK;
}

esp_err_t _bme280_reg_read16(BME280_handle_t bme, uint8_t reg, uint16_t *data)
{
    uint8_t temp_data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, BME_WRITE(bme->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(BME280_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, BME_READ(bme->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_read(cmd, temp_data, 2, LAST_NACK_VAL))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(BME280_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    *data = (temp_data[0] << 8) | temp_data[1];

    return ESP_OK;
}

esp_err_t _bme280_reg_read24(BME280_handle_t bme, uint8_t reg, uint32_t *data)
{

    uint8_t temp_data[3];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, BME_WRITE(bme->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(BME280_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, BME_READ(bme->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_read(cmd, temp_data, 3, LAST_NACK_VAL))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(BME280_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    *data = temp_data[0];
    *data <<= 8;
    *data |= temp_data[1];
    *data <<= 8;
    *data |= temp_data[2];

    return ESP_OK;
}

esp_err_t _bme280_reg_readS16(BME280_handle_t bme, uint8_t reg, int16_t *data)
{
    return _bme280_reg_read16(bme, reg, (uint16_t*)data);
}

esp_err_t _bme280_reg_read16_LE(BME280_handle_t bme, uint8_t reg, uint16_t *data)
{
    uint16_t temp_data;
    esp_err_t ret = _bme280_reg_read16(bme, reg, &temp_data);
    *data = (temp_data >> 8) | (temp_data << 8);
    return ret;
}

esp_err_t _bme280_reg_readS16_LE(BME280_handle_t bme, uint8_t reg, int16_t *data)
{
    return _bme280_reg_read16_LE(bme, reg, (uint16_t*)data);
}

// endregion
