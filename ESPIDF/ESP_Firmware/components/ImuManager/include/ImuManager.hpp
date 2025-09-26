#pragma once
#include <vector>
#include <cstring>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lsm6dsv320x_reg.h"

#define BOOT_TIME 10 // ms
#define POLL_INTERVAL 1000 // ms
#define CNT_FOR_OUTPUT 100

using RawAccelVec = std::vector<float>;
using RawGyroVec = std::vector<float>;
using QuaternionVec = std::vector<float>;
using Temperature = float;
using EulerAngles = double;
using TimeInSeconds = double;
using SpiPins = std::uint8_t;
using SpiMode = std::uint8_t;
using Flag = bool;
using u32 = std::uint32_t;
using u16 = std::uint16_t;
using RawMotion = std::uint16_t;
using RawTemperature = std::uint16_t;
using u8 = std::uint8_t;
using SpiWriteFunc = u32 (*)(void*, u8, const u8*, u16);
using SpiReadFunc = u32 (*)(void*, u8, u8*, u16);
using SpiDelayFunc = void (*)(uint32_t);
using DataReadyStatus = lsm6dsv320x_data_ready_t;
using DevCtx = stmdev_ctx_t;
using SpiDeviceHandle = spi_device_handle_t;
using FilterSettingMask = lsm6dsv320x_filt_settling_mask_t;
using Character = char;

class ImuManager {
private:
    const Character* TAG = "LSM6DSV320X_MANAGER";
    const SpiPins cs_, sck_, miso_, mosi_;
    const SpiMode mode_;
    RawAccelVec lowAccel_{3, 0.0f};
    RawAccelVec highAccel_{3, 0.0f};
    RawGyroVec dpsGyro_{3, 0.0f};
    Temperature tempInC_{0.0f};
    QuaternionVec quat_{4, 0.0f};
    EulerAngles pitch_{0.0}, roll_{0.0}, yaw_{0.0};
    RawMotion dataRawMotion[3];
    RawTemperature dataRawTemperature;
    DataReadyStatus status, lowAccelStatus_, highAccelStatus_, gyroStatus_, tempStatus_;
    DevCtx devCtx;
    SpiDeviceHandle spiHandle;
    FilterSettingMask filterSettingMask;
    u8 whoamI;
    Character txBuffer[256];

    Flag initSpi();
    u32 spiRead(void* handle, u8 reg, u8* bufp, u16 len);
    u32 spiWrite(void* handle, u8 reg, const u8* bufp, u16 len);
    void spidelay(u32 ms);
    void initImu320x(stmdev_ctx_t& devCtx, SpiWriteFunc write, SpiReadFunc read, SpiDelayFunc delay, void* handle);
    Flag whoAmI();
    Flag resetImu();
    void setupImuDataRatesAndScales();
    void setupImuFilter();
    DataReadyStatus getImuDataStatus();
    DataReadyStatus isLowAccelReady() const { return lowAccelStatus_; }
    DataReadyStatus isHighAccelReady() const { return highAccelStatus_; }
    DataReadyStatus isGyroReady() const { return gyroStatus_; }
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
    EulerAngles getRoll() const { return roll_; }
    EulerAngles getYaw() const { return yaw_; }
};

