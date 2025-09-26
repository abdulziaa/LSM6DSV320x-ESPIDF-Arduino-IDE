#pragma once

#include <vector>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lsm6dsv320x_reg.h"


#define BOOT_TIME       10   // ms
#define POLL_INTERVAL   1000   // ms
#define CNT_FOR_OUTPUT  100

using RawAccelVec   = std::vector<float>;
using RawGyroVec    = std::vector<float>;
using QuaternionVec = std::vector<float>;
using Temperature   = float;
using EulerAngles   = double;
using TimeInSeconds = double;
using SpiPins       = std::uint8_t;
using SpiMode       = std::uint8_t;
using Flag          = bool;
using u32           = std::uint32_t;
using u16           = std::uint16_t;
using RawMotion     = std::uint16_t;
using RawTemperature= std::uint16_t;
using u8            = std::uint8_t;
using SpiWriteFunc     = u32(*)(void*, u8, const u8*, u16);
using SpiReadFunc      = u32(*)(void*, u8, u8*, u16);
using SpiDelayFunc     = void(*)(uint32_t);
using DataReadyStatus  = lsm6dsv320x_data_ready_t;
using DevCtx = stmdev_ctx_t;
using SpiDeviceHandle = spi_device_handle_t;
using FilterSettingMask = lsm6dsv320x_filt_settling_mask_t;
using Character = char;



class ImuManager
{
private:
    const Character *TAG = "LSM6DSV320X_MANAGER";
    const SpiPins cs_, sck_, miso_, mosi_;
    const SpiMode mode_;
    RawAccelVec lowAccel_{3, 0.0f};   
    RawAccelVec highAccel_{3, 0.0f};   
    RawGyroVec  dpsGyro_{3, 0.0f};    
    Temperature tempInC_{0.0f};
    QuaternionVec quat_{4, 0.0f};  
    EulerAngles pitch_{0.0}, roll_{0.0}, yaw_{0.0};
    RawMotion dataRawMotion[3];
    RawTemperature dataRawTemperature;
    DataReadyStatus status, lowAccelStatus_, highAccelStatus_, gyroStatus_, tempStatus_;
    DevCtx devCtx;
    SpiDeviceHandle spiHandle;
    static FilterSettingMask filterSettingMask;
    u8 whoamI;
    Character txBuffer[256];

    Flag initSpi();
    static u32 spiRead(void *handle, u8 reg, u8 *bufp, u16 len);
    static u32 spiWrite(void *handle, u8 reg, const u8 *bufp, u16 len);
    static u32 spidelay(u32 ms);

    void initImu320x(stmdev_ctx_t& devCtx, SpiWriteFunc write, SpiReadFunc read, SpiDelayFunc delay, void* handle);
    Flag whoAmI();
    Flag resetImu();
    void setupImuDataRatesAndScales();
    void setupImuFilter();
    DataReadyStatus getImuDataStatus();
    DataReadyStatus isLowAccelReady()    const { return lowAccelStatus_; }
    DataReadyStatus isHighAccelReady()   const { return highAccelStatus_; }
    DataReadyStatus isGyroReady()        const { return gyroStatus_; }
    DataReadyStatus isTemperatureReady() const { return tempStatus_; }

    void updateLowAccelVec();
    void updateHighAccelVec();
    void updateGyroVec();
    void updateTemperatureVar();
    void updateQuaternionVec();
    void updatePitchVar();
    void updateRollVar();
    void updateYawVar();




public:
    ImuManager(SpiPins cs = 5, SpiPins sck = 18, SpiPins miso = 19, SpiPins mosi = 23, SpiMode mode = 0);
    ~ImuManager() = default;

    void setup();
    void loop();


    RawAccelVec getRawLowAccelerationIng() const { return lowAccel_; }
    RawAccelVec getRawHighAccelerationIng() const { return highAccel_; }
    RawGyroVec getRawGyroInMdps() const { return dpsGyro_; }
    Temperature getTemperatureInDegreeC() const { return tempInC_; }
    QuaternionVec getQuaternions() const { return quat_; }
    EulerAngles getPitch() const { return pitch_; }
    EulerAngles getRoll()  const { return roll_; }
    EulerAngles getYaw()   const { return yaw_; }
};


ImuManager::ImuManager(SpiPins cs, SpiPins sck, SpiPins miso, SpiPins mosi, SpiMode mode)
    : cs_{cs}, sck_{sck}, miso_{miso}, mosi_{mosi}, mode_{mode}
{
    
}

void ImuManager::setup()
{
    initSpi();
    initImu320x(*devCtx, spiWrite, spiRead, spidelay, *handle);
    spidelay(BOOT_TIME);
    whoAmI();
    resetImu();
    setupImuDataRatesAndScales();
    setupImuFilter();
}
void ImuManager::loop()
{
    getImuDataStatus();
    updateLowAccelVec();
    updateHighAccelVec();
    updateGyroVec();
    updateTemperatureVar();
}


Flag ImuManager::initSpi()
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi_,
        .miso_io_num = miso_,
        .sclk_io_num = sck_,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    spi_device_interface_config_t devcfg = {
            .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
            .mode = mode_,                          // SPI Mode 0
            .spics_io_num = cs_,
            .queue_size = 1,
        };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spiHandle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI initialized");
}


static u32 ImuManager::spiRead(void *handle, u8 reg, u8 *bufp, u16 len)
{
    uint8_t out[1 + len];
    out[0] = reg | 0x80;   // set MSB = read
    memset(&out[1], 0x00, len);  // dummy bytes (Arduino does this internally)

    uint8_t in[1 + len];

    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .txBuffer = out,
        .rx_buffer = in,
    };

    esp_err_t ret = spi_device_transmit(spiHandle, &t);
    if (ret != ESP_OK) return -1;

    memcpy(bufp, &in[1], len); // skip first byte (address echo)
    return 0;

}

static u32 ImuManager::spiWrite(void *handle, u8 reg, const u8 *bufp, u16 len)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint8_t tx_data[1 + len];
    tx_data[0] = reg & 0x7F;  // clear MSB for write
    memcpy(&tx_data[1], bufp, len);

    t.length = (1 + len) * 8;
    t.txBuffer = tx_data;
    t.flags = SPI_TRANS_USE_TXDATA;

    gpio_set_level(cs_, 0);
    ret = spi_device_transmit(spiHandle, &t);
    gpio_set_level(cs_, 1);

    return (ret == ESP_OK) ? 0 : -1;

}

static u32 ImuManager::spidelay(u32 ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void ImuManager::initImu320x(stmdev_ctx_t& devCtx, SpiWriteFunc write, SpiReadFunc read, SpiDelayFunc delay, void* handle)
{
    devCtx.write_reg = write;
    devCtx.read_reg  = read;
    devCtx.mdelay    = delay;
    devCtx.handle    = handle;
}

Flag ImuManager::whoAmI()
{
    lsm6dsv320x_device_id_get(&devCtx, &whoamI);
    ESP_LOGI(TAG, "WHOAMI: 0x%02X", whoamI);
    if (whoamI != LSM6DSV320X_ID) 
    {
        ESP_LOGE(TAG, "Wrong device ID, expected 0x%02X", LSM6DSV320X_ID);
        return false;
    }
    return true;

}

Flag ImuManager::resetImu()
{
    // Reset sensor
    lsm6dsv320x_reset_t rst;
    lsm6dsv320x_reset_set(&devCtx, LSM6DSV320X_RESTORE_CTRL_REGS);

    do 
    {
        lsm6dsv320x_reset_get(&devCtx, &rst);
    } 
    while (rst != LSM6DSV320X_READY);

}

void setupImuDataRatesAndScales()
{
    // Configure sensor
    lsm6dsv320x_block_data_update_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_xl_data_rate_set(&devCtx, LSM6DSV320X_ODR_AT_60Hz);
    lsm6dsv320x_hg_xl_data_rate_set(&devCtx, LSM6DSV320X_HG_XL_ODR_AT_960Hz, 1);
    lsm6dsv320x_gy_data_rate_set(&devCtx, LSM6DSV320X_ODR_AT_120Hz);
    lsm6dsv320x_xl_full_scale_set(&devCtx, LSM6DSV320X_2g);
    lsm6dsv320x_hg_xl_full_scale_set(&devCtx, LSM6DSV320X_320g);
    lsm6dsv320x_gy_full_scale_set(&devCtx, LSM6DSV320X_2000dps);

}

void ImuManager::setupImuFilter()
{
    // Configure filtering
    filterSettingMask.drdy = PROPERTY_ENABLE;
    filterSettingMask.irq_xl = PROPERTY_ENABLE;
    filterSettingMask.irq_g = PROPERTY_ENABLE;
    lsm6dsv320x_filt_settling_mask_set(&devCtx, filterSettingMask);
    lsm6dsv320x_filt_gy_lp1_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_gy_lp1_bandwidth_set(&devCtx, LSM6DSV320X_GY_ULTRA_LIGHT);
    lsm6dsv320x_filt_xl_lp2_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_xl_lp2_bandwidth_set(&devCtx, LSM6DSV320X_XL_STRONG);

}

DataReadyStatus ImuManager::getImuDataStatus()
{
    lsm6dsv320x_flag_data_ready_get(&devCtx, &status);
    lowAccelStatus_  = status.drdy_xl;
    highAccelStatus_ = status.drdy_hgxl;
    gyroStatus_      = status.drdy_gy;
    tempStatus_      = status.drdy_temp;
}

void ImuManager::updateLowAccelVec()
{
    if (isLowAccelReady()) 
    {
            memset(dataRawMotion, 0, sizeof(dataRawMotion));
            lsm6dsv320x_acceleration_raw_get(&devCtx, dataRawMotion);
            lowAccel_[0] = lsm6dsv320x_from_fs2_to_mg(lowAccel_[0]);
            lowAccel_[1] = lsm6dsv320x_from_fs2_to_mg(lowAccel_[1]);
            lowAccel_[2] = lsm6dsv320x_from_fs2_to_mg(lowAccel_[2]);
            ESP_LOGI(TAG, "LOW Accel [g]: %.3f  %.3f  %.3f", lowAccel_[0] / 1000.0, lowAccel_[1] / 1000.0, lowAccel_[2] / 1000.0);
        }

}

void ImuManager::updateHighAccelVec()
{
    if (isHighAccelReady()) 
        {
        memset(dataRawMotion, 0x00, 3 * sizeof(dataRawMotion));
        lsm6dsv320x_hg_acceleration_raw_get(&devCtx, dataRawMotion);
        highAccel_[0] = lsm6dsv320x_from_fs2_to_mg(dataRawMotion[0]);
        highAccel_[1] = lsm6dsv320x_from_fs2_to_mg(dataRawMotion[1]);
        highAccel_[2] = lsm6dsv320x_from_fs256_to_mg(dataRawMotion[2]);
        ESP_LOGI(TAG, "HIGH Accel [g]: %.3f  %.3f  %.3f", highAccel_[0] / 1000.0, highAccel_[1] / 1000.0, highAccel_[2] / 1000.0);
        }
}

void ImuManager::updateGyroVec()
{
    if (isGyroReady()) 
    {
            memset(dataRawMotion, 0, sizeof(dataRawMotion));
            lsm6dsv320x_angular_rate_raw_get(&devCtx, dataRawMotion);
            dpsGyro_[0] = lsm6dsv320x_from_fs2000_to_mdps(dpsGyro_[0]);
            dpsGyro_[1] = lsm6dsv320x_from_fs2000_to_mdps(dpsGyro_[1]);
            dpsGyro_[2] = lsm6dsv320x_from_fs2000_to_mdps(dpsGyro_[2]);
            ESP_LOGI(TAG, "Gyro [mdps]: %.2f  %.2f  %.2f", dpsGyro_[0]/ 1000.0, dpsGyro_[1]/ 1000.0, dpsGyro_[2]/ 1000.0);
        }

}
void ImuManager::updateTemperatureVar()
{
    if (isTemperatureReady()) 
    {
        memset(&dataRawTemperature, 0, sizeof(dataRawTemperature));
        lsm6dsv320x_temperature_raw_get(&devCtx, &dataRawTemperature);
        tempInC_ = lsm6dsv320x_from_lsb_to_celsius(dataRawTemperature);
        ESP_LOGI(TAG, "Temp [°C]: %.2f", tempInC_);
    }
}

void ImuManager::updateQuaternionVec()
{
 //leave it for now 
}

void ImuManager::updatePitchVar()
{
 //leave it for now 
}

void ImuManager::updateRollVar()
{
//leave it for now 
}

void ImuManager::updateYawVar()
{
//leave it for now 
}