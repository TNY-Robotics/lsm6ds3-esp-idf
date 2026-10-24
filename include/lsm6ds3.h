/**
 * @file lsm6ds3.h
 * @brief LSM6DS3 ESP-IDF Driver Header File
 * 
 * This file contains the definitions, structures, and function prototypes for the LSM6DS3 driver.
 */

/*
 * LSM6DS3 ESP-IDF Driver by TNY Robotics
 *
 * SPDX-FileCopyrightText: 2026 TNY Robotics
 * SPDX-License-Identifier: MIT
 * 
 * Copyright (C) 2026 TNY Robotics
 * 
 * License: MIT
 * Author: TNY Robotics
 * Version: 1.0
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * Note: ST datasheet specifies 0xD5/0xD7 (8-bit address).
 * In 7-bit format for ESP-IDF: SA0=GND -> 0x6A, SA0=VCC -> 0x6B.
 */
#define LSM6DS3_ADDR_SA0_LOW   0x6A
#define LSM6DS3_ADDR_SA0_HIGH  0x6B
#define LSM6DS3_DEFAULT_ADDR   LSM6DS3_ADDR_SA0_LOW
#define LSM6DS3_DEFAULT_CLOCK  400000 /*!< LSM6DS3 supports Fast Mode 400kHz out of the box */

#define LSM6DS3_WHO_AM_I_VAL      0x69 /*!< Standard LSM6DS3 */
#define LSM6DS3TRC_WHO_AM_I_VAL   0x6A /*!< LSM6DS3TR-C hardware revision */

/**
 * @brief LSM6DS3 accelerometer full-scale range options.
 * @note Values match the FS_XL [1:0] bitfield in CTRL1_XL (0x10).
 */
typedef enum {
    ACCEL_FS_2G  = 0, /*!< Accelerometer full scale range is +/- 2g (00b) */
    ACCEL_FS_16G = 1, /*!< Accelerometer full scale range is +/- 16g (01b) */
    ACCEL_FS_4G  = 2, /*!< Accelerometer full scale range is +/- 4g (10b) */
    ACCEL_FS_8G  = 3, /*!< Accelerometer full scale range is +/- 8g (11b) */
} lsm6ds3_accel_fs_t;

/**
 * @brief LSM6DS3 gyroscope full-scale range options.
 * @note Values match the [FS_G1, FS_G0, FS_125] bitfield in CTRL2_G (0x11).
 */
typedef enum {
    GYRO_FS_250DPS  = 0, /*!< Gyroscope full scale range is +/- 250 dps (000b) */
    GYRO_FS_125DPS  = 1, /*!< Gyroscope full scale range is +/- 125 dps (001b) */
    GYRO_FS_500DPS  = 2, /*!< Gyroscope full scale range is +/- 500 dps (010b) */
    GYRO_FS_1000DPS = 4, /*!< Gyroscope full scale range is +/- 1000 dps (100b) */
    GYRO_FS_2000DPS = 6, /*!< Gyroscope full scale range is +/- 2000 dps (110b) */
} lsm6ds3_gyro_fs_t;

/**
 * @brief LSM6DS3 Output Data Rate (ODR) options for Accel and Gyro.
 */
typedef enum {
    LSM6DS3_ODR_POWER_DOWN = 0x00, /*!< Sensor disabled / sleep mode */
    LSM6DS3_ODR_12_5HZ     = 0x01, /*!< 12.5 Hz */
    LSM6DS3_ODR_26HZ       = 0x02, /*!< 26 Hz */
    LSM6DS3_ODR_52HZ       = 0x03, /*!< 52 Hz */
    LSM6DS3_ODR_104HZ      = 0x04, /*!< 104 Hz */
    LSM6DS3_ODR_208HZ      = 0x05, /*!< 208 Hz */
    LSM6DS3_ODR_416HZ      = 0x06, /*!< 416 Hz (Recommended for robotics IMU loops) */
    LSM6DS3_ODR_833HZ      = 0x07, /*!< 833 Hz */
    LSM6DS3_ODR_1660HZ     = 0x08, /*!< 1.66 kHz */
} lsm6ds3_odr_t;

/**
 * @brief LSM6DS3 device information structure.
 */
typedef struct {
    uint16_t address;     /*!< 7-bit I2C address of the LSM6DS3 device (0x6A or 0x6B) */
    uint32_t clock_speed; /*!< I2C clock speed in Hz */
} lsm6ds3_info_t;

/**
 * @brief LSM6DS3 configuration structure.
 */
typedef struct {
    lsm6ds3_accel_fs_t accel_fs; /*!< Accelerometer full-scale range */
    lsm6ds3_gyro_fs_t gyro_fs;   /*!< Gyroscope full-scale range */
    lsm6ds3_odr_t odr;           /*!< Active Output Data Rate when awake */
    bool wake_auto;              /*!< Whether to automatically wake up the device after config */
} lsm6ds3_config_t;

#ifdef __cplusplus
#define LSM6DS3_DEFAULT_INFO() {LSM6DS3_DEFAULT_ADDR, LSM6DS3_DEFAULT_CLOCK}
#define LSM6DS3_DEFAULT_CONFIG() {ACCEL_FS_4G, GYRO_FS_500DPS, LSM6DS3_ODR_416HZ, true}
#else
#define LSM6DS3_DEFAULT_INFO() ((lsm6ds3_info_t){\
    .address = LSM6DS3_DEFAULT_ADDR,\
    .clock_speed = LSM6DS3_DEFAULT_CLOCK\
})
#define LSM6DS3_DEFAULT_CONFIG() ((lsm6ds3_config_t){\
    .accel_fs = ACCEL_FS_4G,\
    .gyro_fs = GYRO_FS_500DPS,\
    .odr = LSM6DS3_ODR_416HZ,\
    .wake_auto = true\
})
#endif

/**
 * @brief LSM6DS3 raw accelerometer values.
 */
typedef struct {
    int16_t raw_accel_x; /*!< Raw accelerometer X-axis value */
    int16_t raw_accel_y; /*!< Raw accelerometer Y-axis value */
    int16_t raw_accel_z; /*!< Raw accelerometer Z-axis value */
} lsm6ds3_raw_accel_value_t;

/**
 * @brief LSM6DS3 raw gyroscope values.
 */
typedef struct {
    int16_t raw_gyro_x; /*!< Raw gyroscope X-axis value */
    int16_t raw_gyro_y; /*!< Raw gyroscope Y-axis value */
    int16_t raw_gyro_z; /*!< Raw gyroscope Z-axis value */
} lsm6ds3_raw_gyro_value_t;

/**
 * @brief LSM6DS3 accelerometer values in g.
 */
typedef struct {
    float accel_x; /*!< Accelerometer X-axis value in g */
    float accel_y; /*!< Accelerometer Y-axis value in g */
    float accel_z; /*!< Accelerometer Z-axis value in g */
} lsm6ds3_accel_value_t;

/**
 * @brief LSM6DS3 gyroscope values in degrees per second.
 */
typedef struct {
    float gyro_x; /*!< Gyroscope X-axis value in degrees per second */
    float gyro_y; /*!< Gyroscope Y-axis value in degrees per second */
    float gyro_z; /*!< Gyroscope Z-axis value in degrees per second */
} lsm6ds3_gyro_value_t;

/**
 * @brief LSM6DS3 temperature value in degrees Celsius.
 */
typedef struct {
    float temp; /*!< Temperature value in degrees Celsius */
} lsm6ds3_temp_value_t;

/**
 * @brief LSM6DS3 device handle structure.
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle; /*!< I2C device handle for LSM6DS3 */
    float accel_sensitivity;            /*!< Accelerometer sensitivity in g/LSB */
    float gyro_sensitivity;             /*!< Gyroscope sensitivity in dps/LSB */
    float temp_sensitivity;             /*!< Temperature sensitivity in °C/LSB */
    lsm6ds3_odr_t target_odr;           /*!< Saved ODR used when waking up from sleep */
} lsm6ds3_handle_t;

/**
 * @brief Create an LSM6DS3 device instance and verify its WHO_AM_I register.
 * 
 * @param i2c_bus_handle Handle to the I2C bus.
 * @param info LSM6DS3 device information.
 * @param out_lsm_handle Pointer to the created LSM6DS3 handle.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_create(i2c_master_bus_handle_t i2c_bus_handle, lsm6ds3_info_t info, lsm6ds3_handle_t* out_lsm_handle);

/**
 * @brief Configure the LSM6DS3 device.
 * 
 * @param lsm_handle LSM6DS3 device handle pointer.
 * @param config Configuration parameters for the LSM6DS3.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_config(lsm6ds3_handle_t* lsm_handle, lsm6ds3_config_t config);

/**
 * @brief Perform a software reset on the LSM6DS3 device.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_reset(lsm6ds3_handle_t lsm_handle);

/**
 * @brief Wake up the LSM6DS3 device from power-down mode using configured ODR.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_wake_up(lsm6ds3_handle_t lsm_handle);

/**
 * @brief Put the LSM6DS3 device into power-down (sleep) mode.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_sleep(lsm6ds3_handle_t lsm_handle);

/**
 * @brief Get raw accelerometer values from the LSM6DS3.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @param raw_accel_value Pointer to store raw accelerometer values.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_get_raw_accel(lsm6ds3_handle_t lsm_handle, lsm6ds3_raw_accel_value_t *const raw_accel_value);

/**
 * @brief Get raw gyroscope values from the LSM6DS3.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @param raw_gyro_value Pointer to store raw gyroscope values.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_get_raw_gyro(lsm6ds3_handle_t lsm_handle, lsm6ds3_raw_gyro_value_t *const raw_gyro_value);

/**
 * @brief Get all raw values (Temp, Gyro, Accel) in a single 14-byte I2C burst read.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @param raw_accel Pointer to store raw accel values (can be NULL).
 * @param raw_gyro Pointer to store raw gyro values (can be NULL).
 * @param raw_temp Pointer to store raw temp value (can be NULL).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_get_raw_all(lsm6ds3_handle_t lsm_handle, lsm6ds3_raw_accel_value_t *raw_accel, lsm6ds3_raw_gyro_value_t *raw_gyro, int16_t *raw_temp);

/**
 * @brief Get accelerometer values in g from the LSM6DS3.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @param accel_value Pointer to store accelerometer values in g.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_get_accel(lsm6ds3_handle_t lsm_handle, lsm6ds3_accel_value_t *const accel_value);

/**
 * @brief Get gyroscope values in degrees per second from the LSM6DS3.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @param gyro_value Pointer to store gyroscope values in degrees per second.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_get_gyro(lsm6ds3_handle_t lsm_handle, lsm6ds3_gyro_value_t *const gyro_value);

/**
 * @brief Get all sensor values (Accel in g, Gyro in dps, Temp in C) in a single I2C burst read.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @param accel_value Pointer to store accelerometer values in g (can be NULL).
 * @param gyro_value Pointer to store gyroscope values in degrees per second (can be NULL).
 * @param temp Pointer to store temperature value in degrees Celsius (can be NULL).
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_get_all(lsm6ds3_handle_t lsm_handle, lsm6ds3_accel_value_t *accel_value, lsm6ds3_gyro_value_t *gyro_value, lsm6ds3_temp_value_t *temp);

/**
 * @brief Get the temperature value in degrees Celsius from the LSM6DS3.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @param out_temp Pointer to store the temperature value in degrees Celsius.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_get_temp(lsm6ds3_handle_t lsm_handle, lsm6ds3_temp_value_t* out_temp);

/**
 * @brief Delete the LSM6DS3 device instance from the I2C bus.
 * 
 * @param lsm_handle LSM6DS3 device handle.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t lsm6ds3_delete(lsm6ds3_handle_t lsm_handle);

#ifdef __cplusplus
}
#endif