/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#pragma once

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESP32-S3-EYE configuration
 */
#if CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR
/**
 * @brief DVP camera sensor configuration
 */
#define EXAMPLE_DVP_SCCB_I2C_SCL_PIN                    5
#define EXAMPLE_DVP_SCCB_I2C_SDA_PIN                    4
#define EXAMPLE_DVP_CAM_SENSOR_RESET_PIN                -1
#define EXAMPLE_DVP_CAM_SENSOR_PWDN_PIN                 -1

/**
 * @brief DVP interface configuration
 */
#define EXAMPLE_DVP_XCLK_PIN                            15
#define EXAMPLE_DVP_PCLK_PIN                            13
#define EXAMPLE_DVP_VSYNC_PIN                           6
#define EXAMPLE_DVP_DE_PIN                              7
#define EXAMPLE_DVP_D0_PIN                              11
#define EXAMPLE_DVP_D1_PIN                              9
#define EXAMPLE_DVP_D2_PIN                              8
#define EXAMPLE_DVP_D3_PIN                              10
#define EXAMPLE_DVP_D4_PIN                              12
#define EXAMPLE_DVP_D5_PIN                              18
#define EXAMPLE_DVP_D6_PIN                              17
#define EXAMPLE_DVP_D7_PIN                              16
#endif /* CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR */

#if CONFIG_EXAMPLE_ENABLE_SPI_CAM_SENSOR
/**
 * @brief SPI camera sensor configuration
 */
#define EXAMPLE_SPI_SCCB_I2C_SCL_PIN                    5
#define EXAMPLE_SPI_SCCB_I2C_SDA_PIN                    4
#define EXAMPLE_SPI_CAM_SENSOR_RESET_PIN                -1
#define EXAMPLE_SPI_CAM_SENSOR_PWDN_PIN                 -1

/**
 * @brief SPI camera sensor configuration
 */
#define EXAMPLE_SPI_CAM_CS_PIN                          6
#define EXAMPLE_SPI_CAM_SCLK_PIN                        13
#define EXAMPLE_SPI_CAM_DATA0_IO_PIN                    16

/**
 * @brief SPI camera sensor clock resource configuration
 */
#define EXAMPLE_SPI_CAM_XCLK_PIN                        15
#endif /* CONFIG_EXAMPLE_ENABLE_SPI_CAM_SENSOR */

#ifdef __cplusplus
}
#endif
