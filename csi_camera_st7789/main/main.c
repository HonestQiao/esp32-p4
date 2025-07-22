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

#include "config.h"

static esp_lcd_panel_handle_t panel_handle = NULL;
static QueueHandle_t display_queue = NULL;
static uint8_t *cam_buffers[NUM_CAM_BUFFERS] = {NULL};
static size_t cam_buffer_size = 0;
static uint16_t *scaled_buffer = NULL;  // 缩放后的缓冲区
static int trans_finished_count = 0;


// 屏幕旋转
static void lcd_rotate(uint16_t rotation)
{
    switch (rotation) {
        case 0:
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, !CAMERA_SELFIE_MODE, false);
            break;
        case 90:
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, CAMERA_SELFIE_MODE, false);
            break;
        case 180:
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, CAMERA_SELFIE_MODE, true);
            break;
        case 270:
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, !CAMERA_SELFIE_MODE, true);
            break;
    }
}

// 初始化ST7789显示屏
static void init_lcd_display(void)
{
    // 配置背光GPIO
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_DISP_BK_LIGHT_OFF_LEVEL);

    // 初始化SPI总线
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_NUM_SCLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = LCD_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_DISP_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 安装面板IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_LCD_DC,
        .cs_gpio_num = LCD_PIN_NUM_LCD_CS,
        .pclk_hz = LCD_DISP_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // 创建ST7789面板
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // 复位并初始化面板
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // 开启背光
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_DISP_BK_LIGHT_ON_LEVEL);
}

// 图像缩放函数
static void scale_image(uint16_t *src, uint16_t *dst)
{
    const int src_width = CSI_MIPI_CSI_DISP_HRES;
    const int src_height = CSI_MIPI_CSI_DISP_VRES;
    const int dst_width = LCD_DISP_H_RES;
    const int dst_height = LCD_DISP_V_RES;

    // 将整个目标图像初始化为0（黑色）
    for (int i = 0; i < dst_width * dst_height; i++) {
        dst[i] = 0x00;
    }

    // 计算缩放比例（取宽高比中较小的比例）
    const float width_ratio = (float)dst_width / src_width;
    const float height_ratio = (float)dst_height / src_height;
    const float scale = (width_ratio < height_ratio) ? width_ratio : height_ratio;

    // 计算缩放后的实际尺寸
    const float scaled_width = (int)(src_width * scale);
    const float scaled_height = (int)(src_height * scale);

    // 计算居中偏移量
    const int x_offset = (dst_width - scaled_width) / 2;
    const int y_offset = (dst_height - scaled_height) / 2;

    // 使用最近邻插值进行缩放
    for (int y = 0; y < scaled_height; y++) {
        for (int x = 0; x < scaled_width; x++) {
            // 计算原始图像坐标
            const int src_x = (int)(x / scale);
            const int src_y = (int)(y / scale);

            // 确保坐标不越界
            const int safe_src_x = (src_x < src_width) ? src_x : src_width - 1;
            const int safe_src_y = (src_y < src_height) ? src_y : src_height - 1;

            // 写入目标图像（居中位置）
            dst[(y + y_offset) * dst_width + (x + x_offset)] =
                src[safe_src_y * src_width + safe_src_x];

            /* 若需要交换R/B通道（RGB565->BGR565），使用以下代码：
            uint16_t rgb = src[safe_src_y * src_width + safe_src_x];
            uint16_t r = (rgb >> 11) & 0x1F;
            uint16_t g = (rgb >> 5) & 0x3F;
            uint16_t b = rgb & 0x1F;
            dst[(y + y_offset) * dst_width + (x + x_offset)] =
                (b << 11) | (g << 5) | r;
            */
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
            // 缩放图像
            scale_image((uint16_t*)frame_data, scaled_buffer);

            // 显示到LCD
            esp_lcd_panel_draw_bitmap(panel_handle, 0, 0,
                                     LCD_DISP_H_RES, LCD_DISP_V_RES,
                                     scaled_buffer);
        }
    }
}

void app_main(void)
{
    // 1. 初始化LCD显示屏
    init_lcd_display();
    lcd_rotate(LCD_DISP_ROTATE);

    // 2. 分配缩放缓冲区
    scaled_buffer = heap_caps_malloc(LCD_DISP_H_RES * LCD_DISP_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    assert(scaled_buffer != NULL);

    // 3. 创建显示队列
    display_queue = xQueueCreate(5, sizeof(uint8_t*));
    assert(display_queue != NULL);

    // 4. 创建显示任务
    xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);

    // 5. 初始化CSI摄像头
    cam_buffer_size = CSI_MIPI_CSI_DISP_HRES * CSI_MIPI_CSI_DISP_VRES * 2;  // RGB565: 2 bytes per pixel

    // 初始化MIPI LDO
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_config = {
        .chan_id   = CSI_USED_LDO_CHAN_ID,
        .voltage_mv = CSI_USED_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_config, &ldo_mipi_phy));

    // 分配摄像头帧缓冲区
    size_t frame_buffer_alignment = 0;
    ESP_ERROR_CHECK(esp_cache_get_alignment(0, &frame_buffer_alignment));
    for (int i = 0; i < NUM_CAM_BUFFERS; i++) {
        cam_buffers[i] = heap_caps_aligned_calloc(frame_buffer_alignment, 1,
                                                 cam_buffer_size,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        assert(cam_buffers[i] != NULL);
    }

    // 初始化摄像头传感器
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    example_sensor_handle_t sensor_handle;
#else
    i2c_master_bus_handle_t sensor_handle;
#endif
    example_sensor_config_t sensor_config = {
        .i2c_port_num  = I2C_NUM_0,
        .i2c_sda_io_num = CSI_MIPI_CSI_CAM_SCCB_SDA_IO,
        .i2c_scl_io_num = CSI_MIPI_CSI_CAM_SCCB_SCL_IO,
        .port          = ESP_CAM_SENSOR_MIPI_CSI,
        .format_name   = CSI_CAM_FORMAT,
    };
    example_sensor_init(&sensor_config, &sensor_handle);

    // 初始化CSI控制器
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id                = 0,
        .h_res                  = CSI_MIPI_CSI_DISP_HRES,
        .v_res                  = CSI_MIPI_CSI_DISP_VRES,
        .lane_bit_rate_mbps     = CSI_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num          = 2,
        .byte_swap_en           = false,
        .queue_items            = 1,
    };
    esp_cam_ctlr_handle_t cam_handle = NULL;
    ESP_ERROR_CHECK(esp_cam_new_csi_ctlr(&csi_config, &cam_handle));

    // 注册事件回调
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans   = camera_get_new_buffer,
        .on_trans_finished  = camera_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));

    // 初始化ISP处理器
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz                = 80 * 1000 * 1000,
        .input_data_source     = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet   = false,
        .h_res                 = CSI_MIPI_CSI_DISP_HRES,
        .v_res                 = CSI_MIPI_CSI_DISP_VRES,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    // 6. 启动摄像头捕获
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam_handle));

    // 主循环 - 只需保持运行
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 清理代码（实际不会执行到这里）
    ESP_ERROR_CHECK(esp_cam_ctlr_stop(cam_handle));
    ESP_ERROR_CHECK(esp_cam_ctlr_disable(cam_handle));
    ESP_ERROR_CHECK(esp_cam_ctlr_del(cam_handle));
    ESP_ERROR_CHECK(esp_isp_disable(isp_proc));
    ESP_ERROR_CHECK(esp_isp_del_processor(isp_proc));
    ESP_ERROR_CHECK(esp_ldo_release_channel(ldo_mipi_phy));
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    example_sensor_deinit(sensor_handle);
#endif
    for (int i = 0; i < NUM_CAM_BUFFERS; i++) {
        heap_caps_free(cam_buffers[i]);
    }
    heap_caps_free(scaled_buffer);
    vQueueDelete(display_queue);
}