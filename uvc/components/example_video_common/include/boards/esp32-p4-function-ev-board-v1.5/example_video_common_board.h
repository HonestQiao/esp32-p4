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
 * @brief ESP32-P4-Function-EV-Board V1.5 configuration
 */
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
/**
 * @brief MIPI-CSI camera sensor configuration
 */
#define EXAMPLE_MIPI_CSI_SCCB_I2C_SCL_PIN               8
#define EXAMPLE_MIPI_CSI_SCCB_I2C_SDA_PIN               7
#define EXAMPLE_MIPI_CSI_CAM_SENSOR_RESET_PIN           -1
#define EXAMPLE_MIPI_CSI_CAM_SENSOR_PWDN_PIN            -1
#define EXAMPLE_MIPI_CSI_XCLK_PIN                       -1

#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_MOTOR
#define EXAMPLE_MIPI_CSI_CAM_MOTOR_SCCB_I2C_SCL_PIN     8
#define EXAMPLE_MIPI_CSI_CAM_MOTOR_SCCB_I2C_SDA_PIN     7
#define EXAMPLE_MIPI_CSI_CAM_MOTOR_RESET_PIN            -1
#define EXAMPLE_MIPI_CSI_CAM_MOTOR_PWDN_PIN             -1
#define EXAMPLE_MIPI_CSI_CAM_MOTOR_SIGNAL_PIN           -1
#endif /* CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_MOTOR */
#endif /* CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR */

#if CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR
/**
 * @brief DVP camera sensor configuration
 */
#define EXAMPLE_DVP_SCCB_I2C_SCL_PIN                    8
#define EXAMPLE_DVP_SCCB_I2C_SDA_PIN                    7
#define EXAMPLE_DVP_CAM_SENSOR_RESET_PIN                36
#define EXAMPLE_DVP_CAM_SENSOR_PWDN_PIN                 38

/**
 * @brief DVP interface configuration
 */
#define EXAMPLE_DVP_XCLK_PIN                            20
#define EXAMPLE_DVP_PCLK_PIN                            4
#define EXAMPLE_DVP_VSYNC_PIN                           37
#define EXAMPLE_DVP_DE_PIN                              22
#define EXAMPLE_DVP_D0_PIN                              2
#define EXAMPLE_DVP_D1_PIN                              32
#define EXAMPLE_DVP_D2_PIN                              33
#define EXAMPLE_DVP_D3_PIN                              23
#define EXAMPLE_DVP_D4_PIN                              3
#define EXAMPLE_DVP_D5_PIN                              6
#define EXAMPLE_DVP_D6_PIN                              5
#define EXAMPLE_DVP_D7_PIN                              21
#endif /* CONFIG_EXAMPLE_ENABLE_DVP_CAM_SENSOR */

#if CONFIG_EXAMPLE_ENABLE_SPI_CAM_SENSOR
/**
 * @brief SPI camera sensor configuration
 */
#define EXAMPLE_SPI_SCCB_I2C_SCL_PIN                    8
#define EXAMPLE_SPI_SCCB_I2C_SDA_PIN                    7
#define EXAMPLE_SPI_CAM_SENSOR_RESET_PIN                -1
#define EXAMPLE_SPI_CAM_SENSOR_PWDN_PIN                 -1

/**
 * @brief SPI camera sensor configuration
 */
#define EXAMPLE_SPI_CAM_CS_PIN                          37
#define EXAMPLE_SPI_CAM_SCLK_PIN                        4
#define EXAMPLE_SPI_CAM_DATA0_IO_PIN                    21

/**
 * @brief SPI camera sensor clock resource configuration
 */
#define EXAMPLE_SPI_CAM_XCLK_PIN                        20
#endif /* CONFIG_EXAMPLE_ENABLE_SPI_CAM_SENSOR */
#ifdef __cplusplus
}
#endif
