//
// Created by rarvolt on 03.12.2019.
//

#ifndef THUNDER_DETECT_BME280_H
#define THUNDER_DETECT_BME280_H

#include <stdint.h>
#include <esp_err.h>

/**
 * @brief BME280 address
 */
typedef enum
{
    BME280_ADDR_0 = 0x76,
    BME280_ADDR_1 = 0x77
} BME280_ADDR_t;

/**
 * @brief BME280 oversampling modes
 */
typedef enum
{
    BME280_OV_SKIP = 0x00,
    BME280_OVx1 = 0x01,
    BME280_OVx2 = 0x02,
    BME280_OVx4 = 0x03,
    BME280_OVx8 = 0x04,
    BME280_OVx16 = 0x05
} BME280_OV_t;

/**
 * @brief BME280 sensor mode
 */
typedef enum
{
    BME280_MODE_SLEEP = 0x00,
    BME280_MODE_FORCED = 0x01,
    BME280_MODE_NORMAL = 0x03
} BME280_MODE_t;

/**
 * @brief BME280 t_standby in normal mode
 */
typedef enum
{
    BME280_T_SB_0_5 = 0x00,
    BME280_T_SB_62_5,
    BME280_T_SB_125,
    BME280_T_SB_250,
    BME280_T_SB_500,
    BME280_T_SB_1000,
    BME280_T_SB_10,
    BME280_T_SB_20
} BME280_T_SB_t;

/**
 * @brief BME280 IIR filter coefficient
 */
typedef enum
{
    BME280_FILTER_OFF = 0x00,
    BME280_FILTER_2,
    BME280_FILTER_4,
    BME280_FILTER_8,
    BME280_FILTER_16
} BME280_FILTER_t;

typedef struct BME280* BME280_handle_t;

typedef struct
{
    BME280_MODE_t mode;
    BME280_FILTER_t filter;
    BME280_T_SB_t t_standby;
    BME280_OV_t ov_temp;
    BME280_OV_t ov_press;
    BME280_OV_t ov_hum;
} BME280_config_t;


BME280_handle_t bme280_init(const BME280_ADDR_t addr, const BME280_config_t *config);
void bme280_set_config(BME280_handle_t bme, const BME280_config_t *config);

void bme280_soft_reset(BME280_handle_t bme);
void bme280_force_measure(BME280_handle_t bme);
void bme280_read_temperature(BME280_handle_t bme, float *temperature);
void bme280_read_pressure(BME280_handle_t bme, float *pressure);
void bme280_read_humidity(BME280_handle_t bme, float *humidity);

#endif //THUNDER_DETECT_BME280_H
