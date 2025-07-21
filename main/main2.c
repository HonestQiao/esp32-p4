#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "unity.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "esp_ldo_regulator.h"
#include "driver/isp.h"
#include "example_sensor_init.h"
#include "esp_private/esp_cache_private.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/gpio.h"

// 摄像头配置
#define TEST_USED_LDO_CHAN_ID               3
#define TEST_USED_LDO_VOLTAGE_MV            2500
#define TEST_RGB565_BITS_PER_PIXEL          16
#define TEST_MIPI_CSI_LANE_BITRATE_MBPS     200
#define TEST_MIPI_CSI_CAM_SCCB_SCL_IO       8
#define TEST_MIPI_CSI_CAM_SCCB_SDA_IO       7
#define TEST_MIPI_CSI_DISP_HRES             800
#define TEST_MIPI_CSI_DISP_VRES             640
#define TEST_CAM_FORMAT                     "MIPI_2lane_24Minput_RAW8_800x640_50fps"

// 显示屏配置 (ST7789)
#define LCD_HOST  SPI2_HOST
#define EXAMPLE_PIN_NUM_SCLK           4
#define EXAMPLE_PIN_NUM_MOSI           5
#define EXAMPLE_PIN_NUM_MISO           -1
#define EXAMPLE_PIN_NUM_LCD_DC         21
#define EXAMPLE_PIN_NUM_LCD_RST        20
#define EXAMPLE_PIN_NUM_LCD_CS         22
#define EXAMPLE_PIN_NUM_BK_LIGHT       23
#define EXAMPLE_LCD_H_RES              240    // 显示屏水平分辨率
#define EXAMPLE_LCD_V_RES              320    // 显示屏垂直分辨率
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL

// 双缓冲
#define NUM_CAM_BUFFERS 2

static esp_lcd_panel_handle_t panel_handle = NULL;
static QueueHandle_t display_queue = NULL;
static uint8_t *cam_buffers[NUM_CAM_BUFFERS] = {NULL};
static size_t cam_buffer_size = 0;
static uint16_t *scaled_buffer = NULL; // 缩放后的缓冲区
static int trans_finished_count = 0;

// 初始化ST7789显示屏
static void init_lcd_display(void)
{
    // 配置背光GPIO
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);

    // 初始化SPI总线
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 安装面板IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 3,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // 创建ST7789面板
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // 复位并初始化面板
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // 开启背光
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
}

// 图像缩放函数 (800x640 -> 240x320)
static void scale_image(uint16_t *src, uint16_t *dst)
{
    const int src_width = TEST_MIPI_CSI_DISP_HRES;
    const int src_height = TEST_MIPI_CSI_DISP_VRES;
    const int dst_width = EXAMPLE_LCD_H_RES;
    const int dst_height = EXAMPLE_LCD_V_RES;

    const float x_ratio = (float)src_width / dst_width;
    const float y_ratio = (float)src_height / dst_height;

    for (int y = 0; y < dst_height; y++) {
        for (int x = 0; x < dst_width; x++) {
            int src_x = (int)(x * x_ratio);
            int src_y = (int)(y * y_ratio);
            dst[y * dst_width + x] = src[src_y * src_width + src_x];

            // 缩放时交换R和B通道（适用于RGB565转BGR565）
            // uint16_t rgb = src[src_y * src_width + src_x];
            // uint16_t r = (rgb >> 11) & 0x1F;  // 提取R分量
            // uint16_t g = (rgb >> 5) & 0x3F;   // 提取G分量
            // uint16_t b = rgb & 0x1F;          // 提取B分量
            // dst[y * dst_width + x] = (b << 11) | (g << 5) | r;  // 重组为BGR565
        }
    }
}

// 摄像头事件回调
static bool IRAM_ATTR camera_get_new_buffer(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    // esp_rom_printf("[CAM] Get new buffer %d", trans_finished_count);
    static int buffer_index = 0;
    trans->buffer = cam_buffers[buffer_index];
    trans->buflen = cam_buffer_size;
    buffer_index = (buffer_index + 1) % NUM_CAM_BUFFERS;
    return false;
}

static bool IRAM_ATTR camera_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    // esp_rom_printf("[CAM] Trans finished: %d", trans_finished_count);
    // 将帧数据放入队列供显示任务处理
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(display_queue, &trans->buffer, &xHigherPriorityTaskWoken);
    trans_finished_count++;
    return xHigherPriorityTaskWoken == pdTRUE;
}

// 显示处理任务
void display_task(void *arg)
{
    uint8_t *frame_data;
    while (1) {
        if (xQueueReceive(display_queue, &frame_data, portMAX_DELAY)) {
            // 缩放图像 (800x640 -> 240x320)
            scale_image((uint16_t*)frame_data, scaled_buffer);

            // 显示到LCD
            esp_lcd_panel_draw_bitmap(panel_handle, 0, 0,
                                     EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES,
                                     scaled_buffer);
        }
    }
}

void app_main(void)
{
    // 1. 初始化LCD显示屏
    init_lcd_display();

    // 2. 分配缩放缓冲区
    scaled_buffer = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    assert(scaled_buffer != NULL);

    // 3. 创建显示队列
    display_queue = xQueueCreate(5, sizeof(uint8_t*));
    assert(display_queue != NULL);

    // 4. 创建显示任务
    xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);

    // 5. 初始化CSI摄像头
    cam_buffer_size = TEST_MIPI_CSI_DISP_HRES * TEST_MIPI_CSI_DISP_VRES * 2; // RGB565: 2 bytes per pixel

    // 初始化MIPI LDO
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_config = {
        .chan_id   = TEST_USED_LDO_CHAN_ID,
        .voltage_mv = TEST_USED_LDO_VOLTAGE_MV,
    };
    TEST_ESP_OK(esp_ldo_acquire_channel(&ldo_config, &ldo_mipi_phy));

    // 分配摄像头帧缓冲区
    size_t frame_buffer_alignment = 0;
    TEST_ESP_OK(esp_cache_get_alignment(0, &frame_buffer_alignment));
    for (int i = 0; i < NUM_CAM_BUFFERS; i++) {
        cam_buffers[i] = heap_caps_aligned_calloc(frame_buffer_alignment, 1,
                                                 cam_buffer_size,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        assert(cam_buffers[i] != NULL);
    }

    // 初始化摄像头传感器
    example_sensor_handle_t sensor_handle;
    example_sensor_config_t sensor_config = {
        .i2c_port_num  = I2C_NUM_0,
        .i2c_sda_io_num = TEST_MIPI_CSI_CAM_SCCB_SDA_IO,
        .i2c_scl_io_num = TEST_MIPI_CSI_CAM_SCCB_SCL_IO,
        .port          = ESP_CAM_SENSOR_MIPI_CSI,
        .format_name   = TEST_CAM_FORMAT,
    };
    example_sensor_init(&sensor_config, &sensor_handle);

    // 初始化CSI控制器
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id                = 0,
        .h_res                  = TEST_MIPI_CSI_DISP_HRES,
        .v_res                  = TEST_MIPI_CSI_DISP_VRES,
        .lane_bit_rate_mbps     = TEST_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num          = 2,
        .byte_swap_en           = false,
        .queue_items            = 1,
    };
    esp_cam_ctlr_handle_t cam_handle = NULL;
    TEST_ESP_OK(esp_cam_new_csi_ctlr(&csi_config, &cam_handle));

    // 注册事件回调
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans   = camera_get_new_buffer,
        .on_trans_finished  = camera_trans_finished,
    };
    TEST_ESP_OK(esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL));
    TEST_ESP_OK(esp_cam_ctlr_enable(cam_handle));

    // 初始化ISP处理器
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz                = 80 * 1000 * 1000,
        .input_data_source     = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet   = false,
        .h_res                 = TEST_MIPI_CSI_DISP_HRES,
        .v_res                 = TEST_MIPI_CSI_DISP_VRES,
    };
    TEST_ESP_OK(esp_isp_new_processor(&isp_config, &isp_proc));
    TEST_ESP_OK(esp_isp_enable(isp_proc));

    // 6. 启动摄像头捕获
    TEST_ESP_OK(esp_cam_ctlr_start(cam_handle));

    // 主循环 - 只需保持运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 清理代码（实际不会执行到这里）
    TEST_ESP_OK(esp_cam_ctlr_stop(cam_handle));
    TEST_ESP_OK(esp_cam_ctlr_disable(cam_handle));
    TEST_ESP_OK(esp_cam_ctlr_del(cam_handle));
    TEST_ESP_OK(esp_isp_disable(isp_proc));
    TEST_ESP_OK(esp_isp_del_processor(isp_proc));
    TEST_ESP_OK(esp_ldo_release_channel(ldo_mipi_phy));
    example_sensor_deinit(sensor_handle);
    for (int i = 0; i < NUM_CAM_BUFFERS; i++) {
        heap_caps_free(cam_buffers[i]);
    }
    heap_caps_free(scaled_buffer);
    vQueueDelete(display_queue);
}