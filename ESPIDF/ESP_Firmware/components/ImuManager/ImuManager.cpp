#include "ImuManager.hpp"

// Static member initialization
const Character* ImuManager::TAG = "LSM6DSV320X_MANAGER";
FilterSettingMask ImuManager::filterSettingMask = {0};

ImuManager::ImuManager(gpio_num_t cs, gpio_num_t sck, gpio_num_t miso, gpio_num_t mosi, SpiMode mode)
    : cs_{cs}, sck_{sck}, miso_{miso}, mosi_{mosi}, mode_{mode}, 
      lowAccel_{3, 0.0f}, highAccel_{3, 0.0f}, dpsGyro_{3, 0.0f}, tempInC_{0.0f},
      quat_{4, 0.0f}, pitch_{0.0}, roll_{0.0}, yaw_{0.0},
      dataRawMotion{0}, dataRawTemperature{0}, status{0},
      lowAccelStatus_{false}, highAccelStatus_{false}, gyroStatus_{false}, tempStatus_{false},
      devCtx{0}, spiHandle{0}, whoamI{0}, txBuffer{0} {
    memset(dataRawMotion, 0, sizeof(dataRawMotion));
    memset(&status, 0, sizeof(status));
    memset(txBuffer, 0, sizeof(txBuffer));
    memset(&devCtx, 0, sizeof(devCtx));
    memset(&spiHandle, 0, sizeof(spiHandle));
}

void ImuManager::setup() {
    if (!initSpi()) {
        ESP_LOGE(TAG, "SPI initialization failed");
        return;
    }
    initImu320x(devCtx, &ImuManager::spiWrite, &ImuManager::spiRead, &ImuManager::spidelay, this);
    spidelay(BOOT_TIME);
    if (!whoAmI()) {
        ESP_LOGE(TAG, "IMU identification failed");
        return;
    }
    if (!resetImu()) {
        ESP_LOGE(TAG, "IMU reset failed");
        return;
    }
    setupImuDataRatesAndScales();
    setupImuFilter();
}

void ImuManager::loop() {
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

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 10 * 1000 * 1000; // 10 MHz
    devcfg.mode = mode_;
    devcfg.spics_io_num = cs_;
    devcfg.queue_size = 1;

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return false;
    }
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spiHandle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device addition failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "SPI initialized");
    return true;
}

int32_t ImuManager::spiRead(void* handle, u8 reg, u8* bufp, u16 len) {
    auto* self = static_cast<ImuManager*>(handle);

    uint8_t out[1 + len];
    out[0] = reg | 0x80;
    memset(&out[1], 0x00, len);

    uint8_t in[1 + len];

    spi_transaction_t t = {};
    t.length = (1 + len) * 8;
    t.tx_buffer = out;
    t.rx_buffer = in;

    esp_err_t ret = spi_device_transmit(self->spiHandle, &t);
    if (ret != ESP_OK) return -1;

    memcpy(bufp, &in[1], len);
    return 0;
}

int32_t ImuManager::spiWrite(void* handle, u8 reg, const u8* bufp, u16 len) {
    auto* self = static_cast<ImuManager*>(handle);

    esp_err_t ret;
    spi_transaction_t t = {};
    
    uint8_t tx_data[1 + len];
    tx_data[0] = reg & 0x7F;
    memcpy(&tx_data[1], bufp, len);

    t.length = (1 + len) * 8;
    t.tx_buffer = tx_data;

    gpio_set_level(self->cs_, 0);
    ret = spi_device_transmit(self->spiHandle, &t);
    gpio_set_level(self->cs_, 1);

    return (ret == ESP_OK) ? 0 : -1;
}

void ImuManager::spidelay(u32 ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void ImuManager::initImu320x(stmdev_ctx_t& devCtx, SpiWriteFunc write, SpiReadFunc read, SpiDelayFunc delay, void* handle) {
    devCtx.write_reg = write;
    devCtx.read_reg = read;
    devCtx.mdelay = delay;
    devCtx.handle = handle;
}

Flag ImuManager::whoAmI() {
    lsm6dsv320x_device_id_get(&devCtx, &whoamI);
    ESP_LOGI(TAG, "WHOAMI: 0x%02X", whoamI);
    if (whoamI != LSM6DSV320X_ID) {
        ESP_LOGE(TAG, "Wrong device ID, expected 0x%02X", LSM6DSV320X_ID);
        return false;
    }
    return true;
}

Flag ImuManager::resetImu() {
    lsm6dsv320x_reset_t rst;
    lsm6dsv320x_reset_set(&devCtx, LSM6DSV320X_RESTORE_CTRL_REGS);
    do {
        lsm6dsv320x_reset_get(&devCtx, &rst);
    } while (rst != LSM6DSV320X_READY);
    return true;
}

void ImuManager::setupImuDataRatesAndScales() {
    lsm6dsv320x_block_data_update_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_xl_data_rate_set(&devCtx, LSM6DSV320X_ODR_AT_60Hz);
    lsm6dsv320x_hg_xl_data_rate_set(&devCtx, LSM6DSV320X_HG_XL_ODR_AT_960Hz, 1);
    lsm6dsv320x_gy_data_rate_set(&devCtx, LSM6DSV320X_ODR_AT_120Hz);
    lsm6dsv320x_xl_full_scale_set(&devCtx, LSM6DSV320X_2g);
    lsm6dsv320x_hg_xl_full_scale_set(&devCtx, LSM6DSV320X_320g);
    lsm6dsv320x_gy_full_scale_set(&devCtx, LSM6DSV320X_2000dps);
}

void ImuManager::setupImuFilter() {
    filterSettingMask.drdy = PROPERTY_ENABLE;
    filterSettingMask.irq_xl = PROPERTY_ENABLE;
    filterSettingMask.irq_g = PROPERTY_ENABLE;
    lsm6dsv320x_filt_settling_mask_set(&devCtx, filterSettingMask);
    lsm6dsv320x_filt_gy_lp1_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_gy_lp1_bandwidth_set(&devCtx, LSM6DSV320X_GY_ULTRA_LIGHT);
    lsm6dsv320x_filt_xl_lp2_set(&devCtx, PROPERTY_ENABLE);
    lsm6dsv320x_filt_xl_lp2_bandwidth_set(&devCtx, LSM6DSV320X_XL_STRONG);
}

void ImuManager::getImuDataStatus() {
    lsm6dsv320x_flag_data_ready_get(&devCtx, &status);
    lowAccelStatus_ = status.drdy_xl;
    highAccelStatus_ = status.drdy_hgxl;
    gyroStatus_ = status.drdy_gy;
    tempStatus_ = status.drdy_temp;
}

void ImuManager::updateLowAccelVec() {
    if (isLowAccelReady()) {
        memset(dataRawMotion, 0, sizeof(dataRawMotion));
        lsm6dsv320x_acceleration_raw_get(&devCtx, dataRawMotion);
        lowAccel_[0] = lsm6dsv320x_from_fs2_to_mg(dataRawMotion[0]);
        lowAccel_[1] = lsm6dsv320x_from_fs2_to_mg(dataRawMotion[1]);
        lowAccel_[2] = lsm6dsv320x_from_fs2_to_mg(dataRawMotion[2]);
        // ESP_LOGI(TAG, "LOW Accel [g]: %.3f %.3f %.3f", lowAccel_[0] / 1000.0, lowAccel_[1] / 1000.0, lowAccel_[2] / 1000.0);
    }
}

void ImuManager::updateHighAccelVec() {
    if (isHighAccelReady()) {
        memset(dataRawMotion, 0, sizeof(dataRawMotion));
        lsm6dsv320x_hg_acceleration_raw_get(&devCtx, dataRawMotion);
        highAccel_[0] = lsm6dsv320x_from_fs2_to_mg(dataRawMotion[0]);
        highAccel_[1] = lsm6dsv320x_from_fs2_to_mg(dataRawMotion[1]);
        highAccel_[2] = lsm6dsv320x_from_fs256_to_mg(dataRawMotion[2]);
        // ESP_LOGI(TAG, "HIGH Accel [g]: %.3f %.3f %.3f", highAccel_[0] / 1000.0, highAccel_[1] / 1000.0, highAccel_[2] / 1000.0);
    }
}

void ImuManager::updateGyroVec() {
    if (isGyroReady()) {
        memset(dataRawMotion, 0, sizeof(dataRawMotion));
        lsm6dsv320x_angular_rate_raw_get(&devCtx, dataRawMotion);
        dpsGyro_[0] = lsm6dsv320x_from_fs2000_to_mdps(dataRawMotion[0]);
        dpsGyro_[1] = lsm6dsv320x_from_fs2000_to_mdps(dataRawMotion[1]);
        dpsGyro_[2] = lsm6dsv320x_from_fs2000_to_mdps(dataRawMotion[2]);
        // ESP_LOGI(TAG, "Gyro [mdps]: %.2f %.2f %.2f", dpsGyro_[0] / 1000.0, dpsGyro_[1] / 1000.0, dpsGyro_[2] / 1000.0);
    }
}

void ImuManager::updateTemperatureVar() {
    if (isTemperatureReady()) {
        memset(&dataRawTemperature, 0, sizeof(dataRawTemperature));
        lsm6dsv320x_temperature_raw_get(&devCtx, &dataRawTemperature);
        tempInC_ = lsm6dsv320x_from_lsb_to_celsius(dataRawTemperature);
        // ESP_LOGI(TAG, "Temp [°C]: %.2f", tempInC_);
    }
}

void ImuManager::updateQuaternionVec() {
    // TODO: Implement quaternion update logic
}

void ImuManager::updatePitchVar() {
    // TODO: Implement pitch update logic
}

void ImuManager::updateRollVar() {
    // TODO: Implement roll update logic
}

void ImuManager::updateYawVar() {
    // TODO: Implement yaw update logic
}