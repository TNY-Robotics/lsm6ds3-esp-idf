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

#include "lsm6ds3.h"
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "LSM6DS3";

/* LSM6DS3 Core Registers */
#define LSM6DS3_REG_WHO_AM_I   0x0F
#define LSM6DS3_REG_CTRL1_XL   0x10
#define LSM6DS3_REG_CTRL2_G    0x11
#define LSM6DS3_REG_CTRL3_C    0x12
#define LSM6DS3_REG_OUT_TEMP_L 0x20
#define LSM6DS3_REG_OUTX_L_G   0x22
#define LSM6DS3_REG_OUTX_L_XL  0x28

/* CTRL3_C Bit Masks */
#define LSM6DS3_CTRL3_BDU_MASK      (1 << 6) /*!< Block Data Update */
#define LSM6DS3_CTRL3_IF_INC_MASK   (1 << 2) /*!< Register address auto-increment */
#define LSM6DS3_CTRL3_SW_RESET_MASK (1 << 0) /*!< Software Reset */

#define LSM6DS3_I2C_TIMEOUT_MS 100

/* --- Internal Helper Functions --- */

static esp_err_t lsm6ds3_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t write_buf[2] = {reg, val};
    return i2c_master_transmit(dev, write_buf, sizeof(write_buf), LSM6DS3_I2C_TIMEOUT_MS);
}

static esp_err_t lsm6ds3_read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, LSM6DS3_I2C_TIMEOUT_MS);
}

static esp_err_t lsm6ds3_update_reg_bits(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t current_val = 0;
    ESP_RETURN_ON_ERROR(lsm6ds3_read_regs(dev, reg, &current_val, 1), TAG, "Failed to read reg 0x%02X", reg);
    current_val = (current_val & ~mask) | (val & mask);
    return lsm6ds3_write_reg(dev, reg, current_val);
}

static float lsm6ds3_get_accel_sensitivity(lsm6ds3_accel_fs_t fs)
{
    /* ST sensitivities converted from mg/LSB to g/LSB */
    switch (fs) {
        case ACCEL_FS_2G:  return 0.000061f; /* 0.061 mg/LSB */
        case ACCEL_FS_4G:  return 0.000122f; /* 0.122 mg/LSB */
        case ACCEL_FS_8G:  return 0.000244f; /* 0.244 mg/LSB */
        case ACCEL_FS_16G: return 0.000488f; /* 0.488 mg/LSB */
        default:           return 0.000122f;
    }
}

static float lsm6ds3_get_gyro_sensitivity(lsm6ds3_gyro_fs_t fs)
{
    /* ST sensitivities converted from mdps/LSB to dps/LSB */
    switch (fs) {
        case GYRO_FS_125DPS:  return 0.004375f; /* 4.375 mdps/LSB */
        case GYRO_FS_250DPS:  return 0.008750f; /* 8.750 mdps/LSB */
        case GYRO_FS_500DPS:  return 0.017500f; /* 17.50 mdps/LSB */
        case GYRO_FS_1000DPS: return 0.035000f; /* 35.00 mdps/LSB */
        case GYRO_FS_2000DPS: return 0.070000f; /* 70.00 mdps/LSB */
        default:              return 0.017500f;
    }
}

/* --- Public API Functions --- */

esp_err_t lsm6ds3_create(i2c_master_bus_handle_t i2c_bus_handle, lsm6ds3_info_t info, lsm6ds3_handle_t* out_lsm_handle)
{
    ESP_RETURN_ON_FALSE(i2c_bus_handle != NULL && out_lsm_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = info.address,
        .scl_speed_hz = info.clock_speed,
    };

    i2c_master_dev_handle_t dev_handle = NULL;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle), TAG, "Failed to add I2C device");

    /* Verify WHO_AM_I register (0x0F) */
    uint8_t who_am_i = 0;
    esp_err_t ret = lsm6ds3_read_regs(dev_handle, LSM6DS3_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(dev_handle);
        ESP_LOGE(TAG, "Failed to communicate with LSM6DS3 at address 0x%02X", info.address);
        return ret;
    }

    if (who_am_i != LSM6DS3_WHO_AM_I_VAL && who_am_i != LSM6DS3TRC_WHO_AM_I_VAL) {
        i2c_master_bus_rm_device(dev_handle);
        ESP_LOGE(TAG, "Unexpected WHO_AM_I value: 0x%02X (expected 0x69 or 0x6A)", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    out_lsm_handle->dev_handle = dev_handle;
    out_lsm_handle->accel_sensitivity = lsm6ds3_get_accel_sensitivity(ACCEL_FS_4G);
    out_lsm_handle->gyro_sensitivity = lsm6ds3_get_gyro_sensitivity(GYRO_FS_500DPS);
    out_lsm_handle->target_odr = LSM6DS3_ODR_416HZ;

    // Set temperature sensitivity based on device type (LSM6DS3=16 vs LSM6DS3TR-C=256)
    if (who_am_i == LSM6DS3TRC_WHO_AM_I_VAL) {
        out_lsm_handle->temp_sensitivity = 256.0f;
    } else {
        out_lsm_handle->temp_sensitivity = 16.0f;
    }

    ESP_LOGI(TAG, "LSM6DS3 initialized (WHO_AM_I: 0x%02X)", who_am_i);
    return ESP_OK;
}

esp_err_t lsm6ds3_config(lsm6ds3_handle_t* lsm_handle, lsm6ds3_config_t config)
{
    ESP_RETURN_ON_FALSE(lsm_handle != NULL && lsm_handle->dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    /* 1. Enable Block Data Update (BDU) and Address Auto-Increment (IF_INC) in CTRL3_C */
    ESP_RETURN_ON_ERROR(
        lsm6ds3_update_reg_bits(lsm_handle->dev_handle, LSM6DS3_REG_CTRL3_C,
                                LSM6DS3_CTRL3_BDU_MASK | LSM6DS3_CTRL3_IF_INC_MASK,
                                LSM6DS3_CTRL3_BDU_MASK | LSM6DS3_CTRL3_IF_INC_MASK),
        TAG, "Failed to configure CTRL3_C"
    );

    lsm_handle->target_odr = (config.odr == LSM6DS3_ODR_POWER_DOWN) ? LSM6DS3_ODR_104HZ : config.odr;
    lsm6ds3_odr_t active_odr = config.wake_auto ? lsm_handle->target_odr : LSM6DS3_ODR_POWER_DOWN;

    /* 2. Configure Accelerometer: ODR_XL [7:4] | FS_XL [3:2] | BW_XL [1:0] */
    uint8_t ctrl1_xl = ((uint8_t)active_odr << 4) | (((uint8_t)config.accel_fs & 0x03) << 2);
    ESP_RETURN_ON_ERROR(lsm6ds3_write_reg(lsm_handle->dev_handle, LSM6DS3_REG_CTRL1_XL, ctrl1_xl), TAG, "Failed to write CTRL1_XL");

    /* 3. Configure Gyroscope: ODR_G [7:4] | FS_G [3:1] (includes FS_125 at bit 1) */
    uint8_t ctrl2_g = ((uint8_t)active_odr << 4) | (((uint8_t)config.gyro_fs & 0x07) << 1);
    ESP_RETURN_ON_ERROR(lsm6ds3_write_reg(lsm_handle->dev_handle, LSM6DS3_REG_CTRL2_G, ctrl2_g), TAG, "Failed to write CTRL2_G");

    /* 4. Update sensitivities */
    lsm_handle->accel_sensitivity = lsm6ds3_get_accel_sensitivity(config.accel_fs);
    lsm_handle->gyro_sensitivity = lsm6ds3_get_gyro_sensitivity(config.gyro_fs);

    return ESP_OK;
}

esp_err_t lsm6ds3_reset(lsm6ds3_handle_t lsm_handle)
{
    ESP_RETURN_ON_FALSE(lsm_handle.dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ESP_RETURN_ON_ERROR(
        lsm6ds3_update_reg_bits(lsm_handle.dev_handle, LSM6DS3_REG_CTRL3_C, LSM6DS3_CTRL3_SW_RESET_MASK, LSM6DS3_CTRL3_SW_RESET_MASK),
        TAG, "Failed to trigger software reset"
    );

    /* Wait for SW_RESET bit to clear automatically */
    uint8_t ctrl3 = LSM6DS3_CTRL3_SW_RESET_MASK;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(5));
        ESP_RETURN_ON_ERROR(lsm6ds3_read_regs(lsm_handle.dev_handle, LSM6DS3_REG_CTRL3_C, &ctrl3, 1), TAG, "Failed to poll reset status");
        if ((ctrl3 & LSM6DS3_CTRL3_SW_RESET_MASK) == 0) {
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t lsm6ds3_wake_up(lsm6ds3_handle_t lsm_handle)
{
    ESP_RETURN_ON_FALSE(lsm_handle.dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t odr_bits = ((uint8_t)lsm_handle.target_odr & 0x0F) << 4;
    ESP_RETURN_ON_ERROR(lsm6ds3_update_reg_bits(lsm_handle.dev_handle, LSM6DS3_REG_CTRL1_XL, 0xF0, odr_bits), TAG, "Failed to wake accel");
    ESP_RETURN_ON_ERROR(lsm6ds3_update_reg_bits(lsm_handle.dev_handle, LSM6DS3_REG_CTRL2_G, 0xF0, odr_bits), TAG, "Failed to wake gyro");

    return ESP_OK;
}

esp_err_t lsm6ds3_sleep(lsm6ds3_handle_t lsm_handle)
{
    ESP_RETURN_ON_FALSE(lsm_handle.dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    /* Setting ODR [7:4] to 0 puts both Accel and Gyro into Power-Down mode */
    ESP_RETURN_ON_ERROR(lsm6ds3_update_reg_bits(lsm_handle.dev_handle, LSM6DS3_REG_CTRL1_XL, 0xF0, 0x00), TAG, "Failed to sleep accel");
    ESP_RETURN_ON_ERROR(lsm6ds3_update_reg_bits(lsm_handle.dev_handle, LSM6DS3_REG_CTRL2_G, 0xF0, 0x00), TAG, "Failed to sleep gyro");

    return ESP_OK;
}

esp_err_t lsm6ds3_get_raw_accel(lsm6ds3_handle_t lsm_handle, lsm6ds3_raw_accel_value_t *const raw_accel_value)
{
    ESP_RETURN_ON_FALSE(lsm_handle.dev_handle != NULL && raw_accel_value != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");

    uint8_t buf[6];
    ESP_RETURN_ON_ERROR(lsm6ds3_read_regs(lsm_handle.dev_handle, LSM6DS3_REG_OUTX_L_XL, buf, 6), TAG, "Failed to read raw accel");

    /* LSM6DS3 is Little-Endian: L is at buf[0], H is at buf[1] */
    raw_accel_value->raw_accel_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    raw_accel_value->raw_accel_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    raw_accel_value->raw_accel_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    return ESP_OK;
}

esp_err_t lsm6ds3_get_raw_gyro(lsm6ds3_handle_t lsm_handle, lsm6ds3_raw_gyro_value_t *const raw_gyro_value)
{
    ESP_RETURN_ON_FALSE(lsm_handle.dev_handle != NULL && raw_gyro_value != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");

    uint8_t buf[6];
    ESP_RETURN_ON_ERROR(lsm6ds3_read_regs(lsm_handle.dev_handle, LSM6DS3_REG_OUTX_L_G, buf, 6), TAG, "Failed to read raw gyro");

    raw_gyro_value->raw_gyro_x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    raw_gyro_value->raw_gyro_y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    raw_gyro_value->raw_gyro_z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));

    return ESP_OK;
}

esp_err_t lsm6ds3_get_raw_all(lsm6ds3_handle_t lsm_handle, lsm6ds3_raw_accel_value_t *raw_accel, lsm6ds3_raw_gyro_value_t *raw_gyro, int16_t *raw_temp)
{
    ESP_RETURN_ON_FALSE(lsm_handle.dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    /* 
     * Burst read 14 bytes starting from OUT_TEMP_L (0x20):
     * Bytes 0..1  : Temperature (0x20 - 0x21)
     * Bytes 2..7  : Gyroscope X, Y, Z (0x22 - 0x27)
     * Bytes 8..13 : Accelerometer X, Y, Z (0x28 - 0x2D)
     */
    uint8_t buf[14];
    ESP_RETURN_ON_ERROR(lsm6ds3_read_regs(lsm_handle.dev_handle, LSM6DS3_REG_OUT_TEMP_L, buf, sizeof(buf)), TAG, "Failed burst read");

    if (raw_temp != NULL) {
        *raw_temp = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    }

    if (raw_gyro != NULL) {
        raw_gyro->raw_gyro_x = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
        raw_gyro->raw_gyro_y = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
        raw_gyro->raw_gyro_z = (int16_t)((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));
    }

    if (raw_accel != NULL) {
        raw_accel->raw_accel_x = (int16_t)((uint16_t)buf[8]  | ((uint16_t)buf[9]  << 8));
        raw_accel->raw_accel_y = (int16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8));
        raw_accel->raw_accel_z = (int16_t)((uint16_t)buf[12] | ((uint16_t)buf[13] << 8));
    }

    return ESP_OK;
}

esp_err_t lsm6ds3_get_accel(lsm6ds3_handle_t lsm_handle, lsm6ds3_accel_value_t *const accel_value)
{
    ESP_RETURN_ON_FALSE(accel_value != NULL, ESP_ERR_INVALID_ARG, TAG, "accel_value is NULL");

    lsm6ds3_raw_accel_value_t raw;
    ESP_RETURN_ON_ERROR(lsm6ds3_get_raw_accel(lsm_handle, &raw), TAG, "Failed to get raw accel");

    accel_value->accel_x = (float)raw.raw_accel_x * lsm_handle.accel_sensitivity;
    accel_value->accel_y = (float)raw.raw_accel_y * lsm_handle.accel_sensitivity;
    accel_value->accel_z = (float)raw.raw_accel_z * lsm_handle.accel_sensitivity;

    return ESP_OK;
}

esp_err_t lsm6ds3_get_gyro(lsm6ds3_handle_t lsm_handle, lsm6ds3_gyro_value_t *const gyro_value)
{
    ESP_RETURN_ON_FALSE(gyro_value != NULL, ESP_ERR_INVALID_ARG, TAG, "gyro_value is NULL");

    lsm6ds3_raw_gyro_value_t raw;
    ESP_RETURN_ON_ERROR(lsm6ds3_get_raw_gyro(lsm_handle, &raw), TAG, "Failed to get raw gyro");

    gyro_value->gyro_x = (float)raw.raw_gyro_x * lsm_handle.gyro_sensitivity;
    gyro_value->gyro_y = (float)raw.raw_gyro_y * lsm_handle.gyro_sensitivity;
    gyro_value->gyro_z = (float)raw.raw_gyro_z * lsm_handle.gyro_sensitivity;

    return ESP_OK;
}

esp_err_t lsm6ds3_get_temp(lsm6ds3_handle_t lsm_handle, lsm6ds3_temp_value_t* out_temp)
{
    ESP_RETURN_ON_FALSE(lsm_handle.dev_handle != NULL && out_temp != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid arg");

    uint8_t buf[2];
    ESP_RETURN_ON_ERROR(lsm6ds3_read_regs(lsm_handle.dev_handle, LSM6DS3_REG_OUT_TEMP_L, buf, 2), TAG, "Failed to read temp");

    int16_t raw_temp = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    out_temp->temp = ((float)raw_temp / lsm_handle.temp_sensitivity) + 25.0f;

    return ESP_OK;
}

esp_err_t lsm6ds3_get_all(lsm6ds3_handle_t lsm_handle, lsm6ds3_accel_value_t *accel_value, lsm6ds3_gyro_value_t *gyro_value, lsm6ds3_temp_value_t *temp)
{
    lsm6ds3_raw_accel_value_t raw_xl;
    lsm6ds3_raw_gyro_value_t raw_gy;
    int16_t raw_t = 0;

    ESP_RETURN_ON_ERROR(
        lsm6ds3_get_raw_all(lsm_handle,
                            accel_value ? &raw_xl : NULL,
                            gyro_value  ? &raw_gy : NULL,
                            temp        ? &raw_t  : NULL),
        TAG, "Failed to get all raw data"
    );

    if (accel_value != NULL) {
        accel_value->accel_x = (float)raw_xl.raw_accel_x * lsm_handle.accel_sensitivity;
        accel_value->accel_y = (float)raw_xl.raw_accel_y * lsm_handle.accel_sensitivity;
        accel_value->accel_z = (float)raw_xl.raw_accel_z * lsm_handle.accel_sensitivity;
    }

    if (gyro_value != NULL) {
        gyro_value->gyro_x = (float)raw_gy.raw_gyro_x * lsm_handle.gyro_sensitivity;
        gyro_value->gyro_y = (float)raw_gy.raw_gyro_y * lsm_handle.gyro_sensitivity;
        gyro_value->gyro_z = (float)raw_gy.raw_gyro_z * lsm_handle.gyro_sensitivity;
    }

    if (temp != NULL) {
        temp->temp = ((float)raw_t / lsm_handle.temp_sensitivity) + 25.0f;
    }

    return ESP_OK;
}

esp_err_t lsm6ds3_delete(lsm6ds3_handle_t lsm_handle)
{
    ESP_RETURN_ON_FALSE(lsm_handle.dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    return i2c_master_bus_rm_device(lsm_handle.dev_handle);
}