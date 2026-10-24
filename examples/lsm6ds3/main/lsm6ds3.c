/*
 * LSM6DS3 ESP-IDF Driver by TNY Robotics
 *
 * SPDX-FileCopyrightText: 2025 TNY Robotics
 * SPDX-License-Identifier: MIT
 * 
 * 
 * Copyright (C) 2025 TNY Robotics
 * 
 * This file is part of the LSM6DS3 ESP-IDF Driver.
 * 
 * License: MIT
 * Repository: https://github.com/tny-robotics/lsm6ds3
 * 
 * Author: TNY Robotics
 * Date: 19/06/2025
 * Version: 1.0
 */

#include <freertos/FreeRTOS.h>
#include "lsm6ds3.h"

#define I2C_SDA_GPIO GPIO_NUM_5
#define I2C_SCL_GPIO GPIO_NUM_4
#define I2C_HOST I2C_NUM_0

#ifdef __cplusplus
extern "C"
#endif
void app_main(void)
{
    /* I2C CONFIGURATION */

    // i2c bus configuration
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_HOST,               // I2C port number
        .sda_io_num = I2C_SDA_GPIO,         // GPIO number for I2C sda signal
        .scl_io_num = I2C_SCL_GPIO,         // GPIO number for I2C scl signal
        .clk_source = I2C_CLK_SRC_DEFAULT,  // I2C clock source, just use the default
        .glitch_ignore_cnt = 7,             // glitch filter, again, just use the default
        .flags = {
            .enable_internal_pullup = true, // enable internal pullup resistors (oled screen does not have one)
        },
    };

    // Create the i2c bus handle
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    // Create the lsm6ds3 device handle
    lsm6ds3_handle_t lsm_handle;
    ESP_ERROR_CHECK(lsm6ds3_create(i2c_bus_handle, LSM6DS3_DEFAULT_INFO(), &lsm_handle));

    // Configure the lsm6ds3 with default settings
    ESP_ERROR_CHECK(lsm6ds3_config(&lsm_handle, LSM6DS3_DEFAULT_CONFIG()));

    // NOTE : If wake_auto is disabled in the config, use lsm6ds3_wake_up() to wake up the device from sleep mode before reading values.

    // Get the accelerometer, gyroscope and temperature values
    while (true)
    {
        lsm6ds3_accel_value_t raw_accel;
        lsm6ds3_gyro_value_t raw_gyro;
        lsm6ds3_temp_value_t temp;

        ESP_ERROR_CHECK(lsm6ds3_get_accel(lsm_handle, &raw_accel));
        ESP_ERROR_CHECK(lsm6ds3_get_gyro(lsm_handle, &raw_gyro));
        ESP_ERROR_CHECK(lsm6ds3_get_temp(lsm_handle, &temp));

        // Print the values to the console
        printf("Accelerometer: X = %.2f g, Y = %.2f g, Z = %.2f g\n", raw_accel.accel_x, raw_accel.accel_y, raw_accel.accel_z);
        printf("Gyroscope: X = %.2f dps, Y = %.2f dps, Z = %.2f dps\n", raw_gyro.gyro_x, raw_gyro.gyro_y, raw_gyro.gyro_z);
        printf("Temperature: %.2f °C\n", temp.temp);

        vTaskDelay(pdMS_TO_TICKS(1000)); // wait a bit to avoid spamming the console with values
    }

    // Delete the lsm6ds3 device handle
    ESP_ERROR_CHECK(lsm6ds3_delete(lsm_handle));
}