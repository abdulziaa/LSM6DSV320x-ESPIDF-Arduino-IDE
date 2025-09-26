#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lsm6dsv320x_reg.h"
#include "ImuManager.hpp"
// ---------------- Pin Definitions ----------------
#define CS_PIN    5
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23

// ---------------- Globals ----------------
static const char *TAG = "LSM6DSV320X_EXAMPLE";

static stmdev_ctx_t dev_ctx;
static spi_device_handle_t spi_handle;

// Buffers & variables
static int16_t data_raw_motion[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static lsm6dsv320x_filt_settling_mask_t filt_settling_mask;
static uint8_t whoamI;
static char tx_buffer[256];

#define BOOT_TIME       10   // ms
#define POLL_INTERVAL   1000   // ms
#define CNT_FOR_OUTPUT  100



// ---- WRITE ----
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint8_t tx_data[1 + len];
    tx_data[0] = reg & 0x7F;  // clear MSB for write
    memcpy(&tx_data[1], bufp, len);

    t.length = (1 + len) * 8;
    t.tx_buffer = tx_data;
    t.flags = SPI_TRANS_USE_TXDATA;

    gpio_set_level(CS_PIN, 0);
    ret = spi_device_transmit(spi_handle, &t);
    gpio_set_level(CS_PIN, 1);

    return (ret == ESP_OK) ? 0 : -1;
}
// Read registers
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    uint8_t out[1 + len];
    out[0] = reg | 0x80;   // set MSB = read
    memset(&out[1], 0x00, len);  // dummy bytes (Arduino does this internally)

    uint8_t in[1 + len];

    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = out,
        .rx_buffer = in,
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) return -1;

    memcpy(bufp, &in[1], len); // skip first byte (address echo)
    return 0;
}

static void platform_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void tx_com(uint8_t *tx_buffer, uint16_t len) {
    ESP_LOG_BUFFER_HEX("LSM6DSV320X_TX", tx_buffer, len);
}
// ---------------- SPI Init ----------------
static void spi_init(void) {
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 0,                          // SPI Mode 0
        .spics_io_num = CS_PIN,
        .queue_size = 1,
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI initialized");
}

// ---------------- Main App ----------------
void app_main(void) {
    ESP_LOGI(TAG, "App starting...");

    spi_init();

    // Init sensor context
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay;
    dev_ctx.handle    = &spi_handle;

    platform_delay(BOOT_TIME);

    // Check WHOAMI
    lsm6dsv320x_device_id_get(&dev_ctx, &whoamI);
    ESP_LOGI(TAG, "WHOAMI: 0x%02X", whoamI);
    if (whoamI != LSM6DSV320X_ID) {
        ESP_LOGE(TAG, "Wrong device ID, expected 0x%02X", LSM6DSV320X_ID);
        return;
    }

    // Reset sensor
    lsm6dsv320x_reset_t rst;
    lsm6dsv320x_reset_set(&dev_ctx, LSM6DSV320X_RESTORE_CTRL_REGS);
    do {
        lsm6dsv320x_reset_get(&dev_ctx, &rst);
    } while (rst != LSM6DSV320X_READY);

    // Configure sensor
    lsm6dsv320x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv320x_xl_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_60Hz);
    lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_960Hz, 1);
    lsm6dsv320x_gy_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_120Hz);
    lsm6dsv320x_xl_full_scale_set(&dev_ctx, LSM6DSV320X_2g);
    lsm6dsv320x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV320X_320g);
    lsm6dsv320x_gy_full_scale_set(&dev_ctx, LSM6DSV320X_2000dps);

    // Configure filtering
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
    lsm6dsv320x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
    lsm6dsv320x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV320X_GY_ULTRA_LIGHT);
    lsm6dsv320x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV320X_XL_STRONG);

    ESP_LOGI(TAG, "Sensor initialized");

    // ---------------- Loop ----------------
    while (1) {
        lsm6dsv320x_data_ready_t status;
        lsm6dsv320x_flag_data_ready_get(&dev_ctx, &status);

        if (status.drdy_xl) {
            memset(data_raw_motion, 0, sizeof(data_raw_motion));
            lsm6dsv320x_acceleration_raw_get(&dev_ctx, data_raw_motion);
            acceleration_mg[0] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[0]);
            acceleration_mg[1] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[1]);
            acceleration_mg[2] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[2]);
            ESP_LOGI(TAG, "LOW Accel [g]: %.3f  %.3f  %.3f",
                     acceleration_mg[0] / 1000.0,
                     acceleration_mg[1] / 1000.0,
                     acceleration_mg[2] / 1000.0);
        }

        if (status.drdy_hgxl) 
        {
        memset(data_raw_motion, 0x00, 3 * sizeof(int16_t));
        lsm6dsv320x_hg_acceleration_raw_get(&dev_ctx, data_raw_motion);
        acceleration_mg[0] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[0]);
        acceleration_mg[1] = lsm6dsv320x_from_fs2_to_mg(data_raw_motion[1]);
        acceleration_mg[2] = lsm6dsv320x_from_fs256_to_mg(data_raw_motion[2]);
        ESP_LOGI(TAG, "HIGH Accel [g]: %.3f  %.3f  %.3f",
                    acceleration_mg[0] / 1000.0,
                    acceleration_mg[1] / 1000.0,
                    acceleration_mg[2] / 1000.0);
        }

        if (status.drdy_gy) {
            memset(data_raw_motion, 0, sizeof(data_raw_motion));
            lsm6dsv320x_angular_rate_raw_get(&dev_ctx, data_raw_motion);
            angular_rate_mdps[0] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[0]);
            angular_rate_mdps[1] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[1]);
            angular_rate_mdps[2] = lsm6dsv320x_from_fs2000_to_mdps(data_raw_motion[2]);
            ESP_LOGI(TAG, "Gyro [mdps]: %.2f  %.2f  %.2f",
                     angular_rate_mdps[0]/ 1000.0,
                     angular_rate_mdps[1]/ 1000.0,
                     angular_rate_mdps[2]/ 1000.0);
        }

        if (status.drdy_temp) {
            memset(&data_raw_temperature, 0, sizeof(data_raw_temperature));
            lsm6dsv320x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
            temperature_degC = lsm6dsv320x_from_lsb_to_celsius(data_raw_temperature);
            ESP_LOGI(TAG, "Temp [°C]: %.2f", temperature_degC);
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL));
    }
}
